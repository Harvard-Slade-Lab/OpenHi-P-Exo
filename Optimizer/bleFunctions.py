import asyncio
import struct
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# UUID for the characteristic to which we write and read data
CHARACTERISTIC_UUID = '0000ffe1-0000-1000-8000-00805f9b34fb'

START_MARKER = 0xAA
END_MARKER = 0xBB
# The firmware consumes a fixed-size frame; see Controller/Core/Inc/globals.h
# (RX_BYTES 28) and Controller/Core/Src/torque_profile.c (#define PARAM_COUNT 6).
PARAM_COUNT = 6

class DataSender:
    """
    A class to hold the state required for BLE communication.
    """
    def __init__(self):
        self.client = None
        self.bleConnected = False
        self.isRun = True
        self.sendFlag = False
        self.current_params = []

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
        Sends the current parameters to the BLE device.

        Args:
            s (DataSender): Instance of DataSender.
            client (BleakClient): Connected BLE client.
        """
        s.client = client
        while s.isRun:
            if s.sendFlag:
                try:
                    floats = list(s.current_params)

                    # The frame must match the STM32 parser byte for byte
                    # (Controller/Core/Src/main.c, process_ble_data):
                    #     0xAA 0xAA | 6 x float32 little-endian | 0xBB 0xBB  = 28 bytes
                    # The firmware memcpy's the payload straight from offset 2 and checks the
                    # end markers at fixed offsets 26-27, so no header may sit between the
                    # start markers and the first float, and the count must be exactly six.
                    if len(floats) != PARAM_COUNT:
                        print(f"Refusing to send {len(floats)} parameters; the firmware "
                              f"expects exactly {PARAM_COUNT}. Frame not sent.")
                        s.sendFlag = False
                        continue

                    packet = bytearray()
                    packet.append(START_MARKER)
                    packet.append(START_MARKER)
                    packet += b''.join(struct.pack('<f', val) for val in floats)
                    packet.append(END_MARKER)
                    packet.append(END_MARKER)
                    
                    async def _write_chunk_with_retries(chunk, retries=2, backoff=0.05):
                        attempt = 0
                        while True:
                            try:                                
                                await client.write_gatt_char(BleFunctions.CHARACTERISTIC_UUID, chunk, response=True)
                                return
                            except (BleakError, Exception):
                                attempt += 1
                                if attempt > retries:
                                    raise
                                await asyncio.sleep(backoff * (2 ** (attempt - 1)))
                                
                    # Split data into chunks (BLE characteristic typically max 20 bytes)
                    max_chunk_size = 20
                    for i in range(0, len(packet), max_chunk_size):
                        chunk = packet[i:i + max_chunk_size]
                        # await _write_chunk_with_retries(chunk)
                        try:
                            await asyncio.wait_for(
                                client.write_gatt_char(BleFunctions.CHARACTERISTIC_UUID, chunk, response=True),
                                timeout=1.0
                            )
                        except asyncio.TimeoutError:
                            print("Write timeout! Reconnecting...")
                            return
                        except (BleakError, Exception) as e:
                            print(f"Write failed: {e}. Reconnecting")
                            return
                        
                    formatted_params = [f"{value:.2f}" for value in floats]
                    print(f"Data sent: {formatted_params}")

                    s.sendFlag = False
                except Exception as e:
                    print(f"Error during communication: {e}")
            await asyncio.sleep(0.1)

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
                    await BleFunctions.send_signal(s, client)
                    
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
