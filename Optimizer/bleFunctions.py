import asyncio
import itertools
import random
import struct
import threading
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# UUID for the characteristic to which we write and read data
CHARACTERISTIC_UUID = '0000ffe1-0000-1000-8000-00805f9b34fb'

# The Bluetooth protocol (host <-> controller over the HM-10 BLE-UART bridge), little-endian:
#     SOF0 SOF1 VER TYPE SEQ LEN PAYLOAD[LEN] CRC_LO CRC_HI
# CRC-16/CCITT-FALSE over VER..PAYLOAD. The controller answers every frame with a valid
# CRC with an ACK (PAYLOAD = acked TYPE, STATUS; SEQ = the acked SEQ) over USART1 TX
# (HM-10 RXD to PB6), which the host reads as notifications.
SOF0 = 0xA5
SOF1 = 0x5A
VERSION = 0x02
MAX_PAYLOAD = 24
TYPE_PARAMS = 0x01
TYPE_STOP = 0x02
TYPE_PING = 0x03
TYPE_ACK = 0x80
ACK_APPLIED = 0x00
ACK_REJECTED = 0x01  # non-finite parameter values
ACK_UNKNOWN = 0x02   # unknown TYPE or wrong LEN
ACK_FAULT = 0x03     # a controller fault has latched assistance off until reset (PARAMS, PING)
ACK_REASONS = {
    ACK_REJECTED: 'invalid values',
    ACK_UNKNOWN: 'unknown type or wrong length',
    ACK_FAULT: 'controller fault',
}

# The firmware applies exactly six parameters; see Controller/Core/Src/torque_profile.c
# (#define PARAM_COUNT 6).
PARAM_COUNT = 6
CHUNK_SIZE = 20  # BLE characteristic writes of at most 20 bytes

# Commands in the send queue; a lower priority value is sent first.
STOP = 'STOP'
PARAMS = 'PARAMS'
PING = 'PING'
PRIORITY = {STOP: 0, PARAMS: 1, PING: 2}
FRAME_TYPE = {STOP: TYPE_STOP, PARAMS: TYPE_PARAMS, PING: TYPE_PING}

# Operator messages when a frame is never acknowledged (link fault)
FAULT_MESSAGES = {
    STOP: "Stop not acknowledged: use the control-mode switch or the emergency stop",
    PARAMS: "Parameters not applied by the controller: conditions stopped",
    PING: "No reply from the controller: check the BLE link, the HM-10 RXD to PB6 "
          "(USART1_TX) wiring and the controller firmware",
}
# Operator message when the controller answers ACK_FAULT
CONTROLLER_FAULT = ("Controller fault: assistance is off until the controller is reset. "
                    "Reset the controller, then press Start")


def crc16_ccitt(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(frame_type, seq, payload=b''):
    """One protocol frame."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload of {len(payload)} bytes; at most {MAX_PAYLOAD}")
    body = bytes([VERSION, frame_type, seq & 0xFF, len(payload)]) + bytes(payload)
    crc = crc16_ccitt(body)
    return bytes([SOF0, SOF1]) + body + bytes([crc & 0xFF, crc >> 8])


def params_payload(params):
    """The PARAMS payload: the six parameters as float32, in the firmware's order."""
    floats = [float(v) for v in params]
    if len(floats) != PARAM_COUNT:
        raise ValueError(f"{len(floats)} parameters; the firmware expects exactly {PARAM_COUNT}")
    return b''.join(struct.pack('<f', val) for val in floats)


class FrameParser:
    """Reads protocol frames from a byte stream. It searches for SOF0 SOF1, checks VER
    and LEN, waits for the whole frame and checks the CRC; on any failure it drops one
    byte and searches again."""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        """Add received bytes; returns the complete frames as (TYPE, SEQ, PAYLOAD)."""
        self.buffer += data
        frames = []
        while len(self.buffer) >= 2:
            if self.buffer[0] != SOF0 or self.buffer[1] != SOF1:
                del self.buffer[0]
                continue
            if len(self.buffer) < 6:
                break
            length = self.buffer[5]
            if self.buffer[2] != VERSION or length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            end = 6 + length + 2
            if len(self.buffer) < end:
                break
            crc = self.buffer[end - 2] | (self.buffer[end - 1] << 8)
            if crc16_ccitt(self.buffer[2:end - 2]) != crc:
                del self.buffer[0]
                continue
            frames.append((self.buffer[3], self.buffer[4], bytes(self.buffer[6:end - 2])))
            del self.buffer[:end]
        return frames


class Command:
    """A queued command. done is set when it ends: 'applied' (acknowledged), 'rejected'
    (acknowledged with an error status), 'no_ack' or 'superseded'."""
    def __init__(self, kind, params, order, seq):
        self.kind = kind
        self.params = params
        self.order = order
        self.seq = seq
        self.status = None
        self.done = threading.Event()

    def finish(self, status):
        self.status = status
        self.done.set()


class DataSender:
    """
    A class to hold the state required for BLE communication.

    The controller thread queues commands with send_params(), send_stop() and send_ping();
    a single writer task (BleFunctions.send_signal) sends them one at a time, a Stop first,
    waits for each ACK and retransmits. A frame that is never acknowledged is a link fault:
    link_fault holds the operator message and on_link_fault(kind, message) is called.
    """
    def __init__(self):
        self.client = None
        self.bleConnected = False
        self.isRun = True
        self.current_params = []  # parameters last acknowledged by the controller
        self.ack_timeout = 0.5    # [s] wait for the ACK of a frame
        self.retransmissions = 3  # retransmissions of a frame without ACK
        self.write_timeout = 1.0  # [s] per BLE write
        self.link_fault = None
        self.on_link_fault = None
        self.stop_unconfirmed = False  # the last Stop was never acknowledged
        self.controller_faulted = False  # the last PARAMS or PING was answered ACK_FAULT
        self.in_flight = None
        self._queue = []
        self._order = itertools.count()
        # A random first SEQ, so a restarted host does not repeat the SEQ of the last
        # PARAMS the controller applied (the controller also forgets it at PING and STOP)
        self._seq = itertools.count(random.randrange(256))
        self._ack = None
        self._parser = FrameParser()
        self._lock = threading.Lock()

    def send_params(self, params):
        """Queue new parameters; parameters still waiting in the queue are replaced."""
        with self._lock:
            self._drop(PARAMS)
            command = self._push(PARAMS, [float(v) for v in params])
        return self._check_connected(command)

    def send_stop(self):
        """Queue a Stop (zero assistance) ahead of any parameters. Parameters still waiting
        are dropped, and parameters waiting for their ACK are not sent again, so none are
        sent after the Stop."""
        with self._lock:
            self._drop(PARAMS)
            command = next((c for c in self._queue if c.kind == STOP), None)
            if command is None:
                command = self._push(STOP, None)
        return self._check_connected(command)

    def send_ping(self):
        with self._lock:
            command = self._push(PING, None)
        return self._check_connected(command)

    def check_link(self):
        """Send a PING and wait for its ACK; True when it was acknowledged without a fault.
        Clears a previous link fault. After False, controller_faulted tells a latched
        controller fault from a missing ACK."""
        if self.wait(self.send_ping()):
            self.link_fault = None
            return True
        return False

    def wait(self, command):
        """Wait until a command has ended, at most as long as all its retransmissions take;
        True when it was acknowledged as applied."""
        command.done.wait((1 + self.retransmissions) * (self.ack_timeout + 2 * self.write_timeout) + 1.0)
        return command.status == 'applied'

    def next_command(self):
        """Take the next command to write: a Stop first, otherwise in queue order."""
        with self._lock:
            if not self._queue:
                return None
            command = min(self._queue, key=lambda c: (PRIORITY[c.kind], c.order))
            self._queue.remove(command)
            self.in_flight = command
            return command

    def resend_unconfirmed_stop(self):
        """After (re)connecting: a Stop that was never acknowledged, e.g. because the link
        dropped during its writes, is queued again so it is sent first."""
        with self._lock:
            if self.stop_unconfirmed and not any(c.kind == STOP for c in self._queue):
                self._drop(PARAMS)
                self._push(STOP, None)

    def stop_waiting(self):
        with self._lock:
            return any(c.kind == STOP for c in self._queue)

    def idle(self):
        """True when nothing is queued or being sent."""
        with self._lock:
            return not self._queue and self.in_flight is None

    def expect_ack(self, command):
        with self._lock:
            self._ack = (command, None)

    def ack_status(self):
        with self._lock:
            return self._ack[1] if self._ack is not None else None

    def on_notify(self, _sender, data):
        """Notification handler: bytes from the controller (ACK frames)."""
        with self._lock:
            for frame_type, seq, payload in self._parser.feed(bytes(data)):
                if frame_type != TYPE_ACK or len(payload) != 2 or self._ack is None:
                    continue
                command = self._ack[0]
                # ACKs of an earlier frame (e.g. a late ACK) are ignored
                if seq == command.seq and payload[0] == FRAME_TYPE[command.kind]:
                    self._ack = (command, payload[1])

    def complete(self, command, status, reason='', ack=None):
        """End a command; ack is the STATUS byte of its ACK, if one arrived."""
        with self._lock:
            self.in_flight = None
            self._ack = None
            if status == 'applied' and command.kind == PARAMS:
                self.current_params = list(command.params)
            if command.kind == STOP:
                self.stop_unconfirmed = status != 'applied'
            else:  # STOP is applied also while faulted, so only PARAMS and PING tell
                self.controller_faulted = ack == ACK_FAULT
        # Report a fault before waking the waiting thread, which reads link_fault
        if status in ('rejected', 'no_ack'):
            self.fault(command, reason, ack)
        command.finish(status)

    def fault(self, command, reason, ack=None):
        if ack == ACK_FAULT:
            message = CONTROLLER_FAULT
        else:
            message = f"{FAULT_MESSAGES[command.kind]} ({reason})"
        self.link_fault = message
        print(f"LINK FAULT: {message}")
        if self.on_link_fault is not None:
            self.on_link_fault(command.kind, message)

    def _check_connected(self, command):
        """Without a BLE connection the command cannot be acknowledged: a link fault now.
        A Stop stays queued and is sent after reconnecting."""
        if not self.bleConnected:
            if command.kind != STOP:
                with self._lock:
                    if command in self._queue:
                        self._queue.remove(command)
                    self.controller_faulted = False  # not sent: no fault status to tell
                command.finish('no_ack')
            self.fault(command, 'BLE not connected')
        return command

    def _push(self, kind, params):
        command = Command(kind, params, next(self._order), next(self._seq) & 0xFF)
        self._queue.append(command)
        return command

    def _drop(self, kind):
        for command in [c for c in self._queue if c.kind == kind]:
            self._queue.remove(command)
            command.finish('superseded')

class BleFunctions:
    # UUID for the characteristic to which we write and read data
    CHARACTERISTIC_UUID = '0000ffe1-0000-1000-8000-00805f9b34fb'

    @staticmethod
    async def connect_ble_device(address):
        print("Attempting to connect...")
        client = BleakClient(address, timeout=10.0)
        try:
            connected = await client.connect()
            if connected:
                print(f"Successfully connected to {address}")
                return True, client
            else:
                print(f"Failed to connect to {address}")
                return False, None
        except Exception as e:
            print(f"Failed to connect with error: {e}")
            return False, None

    @staticmethod
    async def disconnect_ble_device(client):
        if client.is_connected:
            await client.disconnect()
            print(f'\nDisconnected from Bluetooth Device')

    @staticmethod
    async def send_signal(s, client):
        """
        Subscribes to the controller's ACKs and sends the queued commands one at a time.
        Returns when the connection is lost, to reconnect.

        Args:
            s (DataSender): Instance of DataSender.
            client (BleakClient): Connected BLE client.
        """
        s.client = client
        await client.start_notify(BleFunctions.CHARACTERISTIC_UUID, s.on_notify)
        s.resend_unconfirmed_stop()
        while s.isRun and client.is_connected:
            command = s.next_command()
            if command is None:
                await asyncio.sleep(0.01)
                continue
            await BleFunctions.transmit(s, client, command)

    @staticmethod
    async def transmit(s, client, command):
        """Write one frame in chunks and wait for its ACK; retransmit it (same SEQ) up to
        s.retransmissions times. Parameters are not sent again once a Stop is waiting."""
        if command.kind == PARAMS:
            payload = params_payload(command.params)
        else:
            payload = b''
        frame = encode_frame(FRAME_TYPE[command.kind], command.seq, payload)
        loop = asyncio.get_running_loop()

        for attempt in range(1 + s.retransmissions):
            if command.kind == PARAMS and s.stop_waiting():
                s.complete(command, 'superseded')
                return
            s.expect_ack(command)
            try:
                # Split data into chunks (BLE characteristic typically max 20 bytes)
                for i in range(0, len(frame), CHUNK_SIZE):
                    await asyncio.wait_for(
                        client.write_gatt_char(BleFunctions.CHARACTERISTIC_UUID, frame[i:i + CHUNK_SIZE], response=True),
                        timeout=s.write_timeout
                    )
            except (asyncio.TimeoutError, BleakError, Exception) as e:
                print(f"{command.kind} write failed (attempt {attempt + 1}): {e!r}")
                continue

            deadline = loop.time() + s.ack_timeout
            status = s.ack_status()
            while status is None and loop.time() < deadline:
                if command.kind == PARAMS and s.stop_waiting():
                    break
                await asyncio.sleep(0.005)
                status = s.ack_status()

            if status == ACK_APPLIED:
                if command.kind == PARAMS:
                    print(f"Parameters applied: {[f'{value:.2f}' for value in command.params]}")
                else:
                    print(f"{command.kind} acknowledged")
                s.complete(command, 'applied', ack=status)
                return
            if status is None and command.kind == PARAMS and s.stop_waiting():
                s.complete(command, 'superseded')
                return
            if status is not None:
                reason = ACK_REASONS.get(status, f'ACK status {status}')
                s.complete(command, 'rejected', f'rejected by the controller: {reason}', ack=status)
                return
            print(f"No ACK for {command.kind} (SEQ {command.seq}, attempt {attempt + 1})")

        s.complete(command, 'no_ack', f'no ACK after {1 + s.retransmissions} attempts')

    @staticmethod
    async def ble_task(s):
        """
        Scans for BLE devices, connects to the target device, and manages sending and receiving signals.

        Args:
            s (DataSender): Instance of DataSender.
        """
        while s.isRun:
            devices = await BleakScanner.discover()
            target_device_name_fragment = "HIP_EXO_MAIN"

            try:
                target_device_name_fragment
            except NameError:
                print("Available Bluetooth devices:")
                for device in devices:
                    if device.name is not None:
                        print(f"Device: {device.name} at {device.address}")
                target_device_name_fragment = input("Enter the partial name of the target Bluetooth device: ")

            # Attempt to find the target device
            target_device_address = None
            for device in devices:
                if device.name and target_device_name_fragment in device.name:
                    print(f"Target device found: {device.name} at {device.address}")
                    target_device_address = device.address
                    break

            if target_device_address:
                connected, client = await BleFunctions.connect_ble_device(target_device_address)
                if connected:
                    s.bleConnected = True
                    try:
                        await BleFunctions.send_signal(s, client)
                    except (BleakError, Exception) as e:
                        print(f"BLE link error: {e}. Reconnecting")
                    s.bleConnected = False

                    await BleFunctions.disconnect_ble_device(client)
                else:
                    print("Failed to connect to the device.")
            else:
                print(f"No device containing '{target_device_name_fragment}' in name found.")

            await asyncio.sleep(1)  # Wait before scanning again

    @staticmethod
    def start_ble_task(s):
        """
        Starts the BLE task in an asyncio event loop.

        Args:
            s (DataSender): Instance of DataSender (your main data/logic class).
        """
        asyncio.run(BleFunctions.ble_task(s))
