"""Mock BLE link for the tests: a recording link, a mock controller that implements
the Bluetooth protocol as specified, a mock BLE client and a writer thread."""
import asyncio
import math
import struct
import threading
import time
from unittest import mock

from tests import _stubs  # noqa: F401  (before the optimizer imports)
from bleFunctions import BleFunctions, DataSender


class FakeLink:
    """Stands in for DataSender: records the commands the controller queues, in order."""
    def __init__(self):
        self.commands = []

    def send_params(self, params):
        self.commands.append(('PARAMS', [float(v) for v in params]))

    def send_stop(self):
        self.commands.append(('STOP',))

    def check_link(self):
        return True

    def params_sent(self):
        return [c[1] for c in self.commands if c[0] == 'PARAMS']


def crc16(data):
    """CRC-16/CCITT-FALSE, written separately from the optimizer's."""
    crc = 0xFFFF
    for byte in data:
        for bit in range(8):
            mix = ((crc >> 15) ^ (byte >> (7 - bit))) & 1
            crc = ((crc << 1) & 0xFFFF) ^ (0x1021 if mix else 0)
    return crc


def frame(frame_type, seq, payload=b''):
    body = bytes([0x02, frame_type, seq, len(payload)]) + payload
    crc = crc16(body)
    return bytes([0xA5, 0x5A]) + body + bytes([crc & 0xFF, crc >> 8])


class MockController:
    """The controller side of the protocol: resynchronizing parser, CRC check, ACK, PARAMS
    applied once per SEQ (forgotten at each PING and STOP), STOP always applied.
    mute: frame types that get no ACK. faulted: a latched fault, so PARAMS is not applied
    and PARAMS and PING are answered 0x03."""
    def __init__(self, mute=()):
        self.mute = set(mute)
        self.faulted = False
        self.buffer = bytearray()
        self.last_params_seq = None
        self.applied = []  # ('PARAMS', params), ('STOP',), ('PING',) in the order applied

    def receive(self, data):
        """Bytes from the HM-10; returns the ACK bytes to send back."""
        self.buffer += data
        out = b''
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b'\xa5\x5a':
                del self.buffer[0]
                continue
            if len(self.buffer) < 6:
                break
            if self.buffer[2] != 0x02 or self.buffer[5] > 24:
                del self.buffer[0]
                continue
            end = 8 + self.buffer[5]
            if len(self.buffer) < end:
                break
            if crc16(self.buffer[2:end - 2]) != self.buffer[end - 2] | (self.buffer[end - 1] << 8):
                del self.buffer[0]
                continue
            frame_type, seq, payload = self.buffer[3], self.buffer[4], bytes(self.buffer[6:end - 2])
            del self.buffer[:end]
            status = self.apply(frame_type, seq, payload)
            if frame_type not in self.mute:
                out += frame(0x80, seq, bytes([frame_type, status]))
        return out

    def apply(self, frame_type, seq, payload):
        if frame_type == 0x01 and len(payload) == 24:
            if self.faulted:
                return 0x03
            params = list(struct.unpack('<6f', payload))
            if not all(math.isfinite(v) for v in params):
                return 0x01
            if seq != self.last_params_seq:
                self.last_params_seq = seq
                self.applied.append(('PARAMS', params))
            return 0x00
        if frame_type == 0x02 and not payload:
            self.last_params_seq = None
            self.applied.append(('STOP',))
            return 0x00
        if frame_type == 0x03 and not payload:
            self.last_params_seq = None
            self.applied.append(('PING',))
            return 0x03 if self.faulted else 0x00
        return 0x02


class MockClient:
    """A BLE client connected to a MockController. Each write takes write_delay seconds;
    chunks whose write index is in lose are accepted but never delivered; ACKs arrive as
    notifications after ack_delay seconds. The link drops at the write with index
    drop_at: that write and every later one fail."""
    def __init__(self, controller=None, write_delay=0.0, lose=(), ack_delay=0.0, drop_at=None):
        self.controller = controller if controller is not None else MockController()
        self.write_delay = write_delay
        self.lose = set(lose)
        self.ack_delay = ack_delay
        self.drop_at = drop_at
        self.is_connected = True
        self.chunks = []  # every chunk written
        self.notify = None
        self.writing = threading.Event()  # set while a write is in progress

    async def start_notify(self, uuid, handler):
        self.notify = handler

    async def write_gatt_char(self, uuid, data, response=True):
        self.writing.set()
        try:
            if self.write_delay:
                await asyncio.sleep(self.write_delay)
            index = len(self.chunks)
            if self.drop_at is not None and index >= self.drop_at:
                self.is_connected = False
                raise ConnectionError("BLE link dropped")
            self.chunks.append(bytes(data))
            if index in self.lose:
                return
            ack = self.controller.receive(bytes(data))
            if ack:
                asyncio.get_running_loop().call_later(self.ack_delay, self.notify, None, bytearray(ack))
        finally:
            self.writing.clear()


class LinkThread:
    """Runs the writer task of a DataSender against a MockClient in its own thread.
    quiet: silence the writer's console lines (default)."""
    def __init__(self, client, sender=None, quiet=True):
        self.client = client
        self.sender = sender if sender is not None else DataSender()
        self.sender.bleConnected = True
        self.sender.isRun = True
        self.quiet = quiet
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        asyncio.run(BleFunctions.send_signal(self.sender, self.client))

    def __enter__(self):
        if self.quiet:
            self._quiet = mock.patch('bleFunctions.print', create=True)  # the writer's console lines
            self._quiet.start()
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.sender.isRun = False
        self.thread.join(timeout=5.0)
        if self.quiet:
            self._quiet.stop()

    def drain(self, timeout=5.0):
        """Wait until every queued command has been sent and acknowledged or given up."""
        end = time.monotonic() + timeout
        while not self.sender.idle():
            if time.monotonic() > end:
                raise AssertionError("the writer did not finish the queued commands")
            time.sleep(0.005)

    def received(self):
        """The PARAMS and STOP commands the controller applied, in order."""
        return [c for c in self.client.controller.applied if c[0] != 'PING']
