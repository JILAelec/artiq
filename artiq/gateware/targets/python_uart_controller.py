#!/usr/bin/env python3
"""
LTC2000 UART Controller

Python interface for controlling LTC2000 DDSs via UART interface.
Provides high-level API for writing coefficients and reading status.

Usage:
    from ltc2000_uart_controller import LTC2000UARTController

    controller = LTC2000UARTController('/dev/ttyUSB0')

    # Test connection
    if controller.ping(0):
        print("Connection OK")

        # Write coefficients to DDS 0
        success = controller.write_coefficients(
            dds_addr=0,
            amp=1000,
            damp=100,
            ftw=1000000,
            shift=2
        )

        if success:
            print("Write successful")

            # Read back status
            status = controller.read_status(0)
            print(f"Status: {status}")

Protocol:
    Command Packet:  [0xAA 0x55 0xCC] [CMD] [ADDR] [30 DATA BYTES] [CRC]
    Response Packet: [0xAA 0x55 0xDD] [STATUS] [DATA...] [CRC]

    Commands:
        0x10: WRITE_COEFF - Write coefficient data
        0x20: READ_COEFF - Read coefficient data
        0x30: READ_STATUS - Read current DDS status
        0x01: PING - Connection test

    Status Codes:
        0x00: OK
        0x80: DATA (response contains data)
        0xFF: ERROR
"""

import serial
import time
import struct
from typing import Optional, Dict, Any, Union

class LTC2000UARTController:
    """Python class to control LTC2000 DDSs via UART"""

    # Command definitions
    CMD_PING = 0x01
    CMD_WRITE_COEFF = 0x10
    CMD_READ_COEFF = 0x20
    CMD_READ_STATUS = 0x30

    # Response codes
    RESP_OK = 0x00
    RESP_DATA = 0x80
    RESP_ERROR = 0xFF

    # Sync patterns
    SYNC_CMD = [0xAA, 0x55, 0xCC]
    SYNC_RESP = [0xAA, 0x55, 0xDD]

    def __init__(self, port: str, baud_rate: int = 115200, timeout: float = 1.0):
        """
        Initialize UART controller.

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0', 'COM3')
            baud_rate: UART baud rate (default: 115200)
            timeout: Read timeout in seconds (default: 1.0)
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self._connect()

    def _connect(self):
        """Establish serial connection"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(0.1)  # Allow connection to stabilize
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to open serial port {self.port}: {e}")

    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def _calculate_crc(self, data: bytes) -> int:
        """Calculate simple XOR checksum"""
        crc = 0
        for byte in data:
            crc ^= byte
        return crc & 0xFF

    def _send_command(self, cmd: int, addr: int, data: Optional[bytes] = None) -> bool:
        """
        Send command packet.

        Args:
            cmd: Command byte
            addr: Address byte (DDS index 0-3)
            data: Optional data payload (padded/truncated to 30 bytes)

        Returns:
            True if packet sent successfully
        """
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("Serial port not open")

        # Prepare data payload (always 30 bytes)
        if data is None:
            data = bytes(30)
        elif len(data) < 30:
            data = data + bytes(30 - len(data))  # Pad
        elif len(data) > 30:
            data = data[:30]  # Truncate

        # Build packet: sync + cmd + addr + data
        packet = bytes(self.SYNC_CMD + [cmd, addr]) + data

        # Add CRC
        crc = self._calculate_crc(packet[3:])  # Skip sync pattern
        packet += bytes([crc])

        try:
            self.ser.write(packet)
            self.ser.flush()
            return True
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            return False

    def _read_response(self) -> tuple[Optional[int], Optional[bytes]]:
        """
        Read response packet.

        Returns:
            Tuple of (status_code, data) or (None, None) on error
        """
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("Serial port not open")

        try:
            # Look for sync pattern
            sync_found = False
            for _ in range(100):  # Timeout after 100 attempts
                byte = self.ser.read(1)
                if not byte:
                    return None, None

                if byte[0] == self.SYNC_RESP[0]:
                    # Found first sync byte, check for complete pattern
                    remaining = self.ser.read(2)
                    if len(remaining) == 2 and list(remaining) == self.SYNC_RESP[1:]:
                        sync_found = True
                        break

            if not sync_found:
                print("Sync pattern not found")
                return None, None

            # Read status byte
            status_byte = self.ser.read(1)
            if not status_byte:
                print("No status byte received")
                return None, None

            status = status_byte[0]

            # Read data based on status
            if status == self.RESP_DATA:
                # DATA response - read more data
                data = self.ser.read(31)  # 30 data + 1 CRC
                if len(data) < 31:
                    print(f"Incomplete data response: got {len(data)} bytes")
                    return status, None
                return status, data[:-1]  # Return without CRC
            else:
                # Status response - read minimal data
                data = self.ser.read(3)   # addr + cmd + CRC
                if len(data) < 3:
                    print(f"Incomplete status response: got {len(data)} bytes")
                    return status, None
                return status, data[:-1]  # Return without CRC

        except serial.SerialException as e:
            print(f"Error reading response: {e}")
            return None, None

    def ping(self, dds_addr: int = 0) -> bool:
        """
        Test connection to specified DDS.

        Args:
            dds_addr: DDS address (0-3)

        Returns:
            True if ping successful
        """
        if not (0 <= dds_addr <= 3):
            raise ValueError("DDS address must be 0-3")

        success = self._send_command(self.CMD_PING, dds_addr)
        if not success:
            return False

        status, data = self._read_response()
        return status == self.RESP_OK

    def write_coefficients(self, dds_addr: int, amp: int = 0, damp: int = 0,
                          ddamp: int = 0, dddamp: int = 0, phase: int = 0,
                          ftw: int = 0, chirp: int = 0, shift: int = 0) -> bool:
        """
        Write coefficient data to specified DDS.

        Args:
            dds_addr: DDS address (0-3)
            amp: Amplitude offset (16-bit signed)
            damp: First derivative (32-bit signed)
            ddamp: Second derivative (48-bit signed)
            dddamp: Third derivative (48-bit signed)
            phase: Phase offset (18-bit unsigned)
            ftw: Frequency tuning word (32-bit unsigned)
            chirp: Chirp rate (32-bit signed)
            shift: Update rate control (4-bit unsigned, 0-15)

        Returns:
            True if write successful
        """
        if not (0 <= dds_addr <= 3):
            raise ValueError("DDS address must be 0-3")

        if not (0 <= shift <= 15):
            raise ValueError("Shift must be 0-15")

        if not (0 <= phase < (1 << 18)):
            raise ValueError(f"Phase must be 0-{(1<<18)-1}")

        # Validate ranges for signed values
        if not (-(1 << 15) <= amp < (1 << 15)):
            raise ValueError(f"Amplitude must be in range {-(1<<15)} to {(1<<15)-1}")

        if not (-(1 << 31) <= damp < (1 << 31)):
            raise ValueError(f"damp must be in range {-(1<<31)} to {(1<<31)-1}")

        if not (-(1 << 47) <= ddamp < (1 << 47)):
            raise ValueError(f"ddamp must be in range {-(1<<47)} to {(1<<47)-1}")

        if not (-(1 << 47) <= dddamp < (1 << 47)):
            raise ValueError(f"dddamp must be in range {-(1<<47)} to {(1<<47)-1}")

        if not (0 <= ftw < (1 << 32)):
            raise ValueError(f"FTW must be in range 0 to {(1<<32)-1}")

        if not (-(1 << 31) <= chirp < (1 << 31)):
            raise ValueError(f"chirp must be in range {-(1<<31)} to {(1<<31)-1}")

        # Pack coefficient data (little-endian format)
        data = bytearray(30)

        # Amplitude offset (16 bits)
        data[0:2] = struct.pack('<h', amp)

        # damp (32 bits)
        data[2:6] = struct.pack('<i', damp)

        # ddamp (48 bits) - pack as 6 bytes
        if ddamp >= 0:
            ddamp_bytes = ddamp.to_bytes(6, 'little', signed=False)
        else:
            ddamp_bytes = (ddamp + (1 << 48)).to_bytes(6, 'little', signed=False)
        data[6:12] = ddamp_bytes

        # dddamp (48 bits) - pack as 6 bytes
        if dddamp >= 0:
            dddamp_bytes = dddamp.to_bytes(6, 'little', signed=False)
        else:
            dddamp_bytes = (dddamp + (1 << 48)).to_bytes(6, 'little', signed=False)
        data[12:18] = dddamp_bytes

        # Phase MSB word (16 bits) - upper 16 bits of 18-bit phase
        phase_msb = (phase >> 2) & 0xFFFF
        data[18:20] = struct.pack('<H', phase_msb)

        # FTW (32 bits)
        data[20:24] = struct.pack('<I', ftw)

        # Chirp (32 bits)
        data[24:28] = struct.pack('<i', chirp)

        # Control word (16 bits) - shift in lower 4 bits, phase LSBs in bits 4-5
        phase_lsb = phase & 0x3
        control = shift | (phase_lsb << 4)
        data[28:30] = struct.pack('<H', control)

        # Send command
        success = self._send_command(self.CMD_WRITE_COEFF, dds_addr, data)
        if not success:
            return False

        status, resp_data = self._read_response()
        return status == self.RESP_OK

    def read_coefficients(self, dds_addr: int) -> Optional[Dict[str, Any]]:
        """
        Read coefficient data from specified DDS.

        Args:
            dds_addr: DDS address (0-3)

        Returns:
            Dictionary with coefficient values or None on error
        """
        if not (0 <= dds_addr <= 3):
            raise ValueError("DDS address must be 0-3")

        success = self._send_command(self.CMD_READ_COEFF, dds_addr)
        if not success:
            return None

        status, data = self._read_response()
        if status != self.RESP_DATA or not data or len(data) < 30:
            return None

        # Unpack coefficient data (little-endian format)
        try:
            amp = struct.unpack('<h', data[0:2])[0]
            damp = struct.unpack('<i', data[2:6])[0]

            # ddamp (48 bits)
            ddamp_bytes = data[6:12]
            ddamp = int.from_bytes(ddamp_bytes, 'little', signed=False)
            if ddamp >= (1 << 47):
                ddamp -= (1 << 48)  # Convert to signed

            # dddamp (48 bits)
            dddamp_bytes = data[12:18]
            dddamp = int.from_bytes(dddamp_bytes, 'little', signed=False)
            if dddamp >= (1 << 47):
                dddamp -= (1 << 48)  # Convert to signed

            phase_msb = struct.unpack('<H', data[18:20])[0]
            ftw = struct.unpack('<I', data[20:24])[0]
            chirp = struct.unpack('<i', data[24:28])[0]
            control = struct.unpack('<H', data[28:30])[0]

            # Extract shift and phase LSBs from control word
            shift = control & 0xF
            phase_lsb = (control >> 4) & 0x3
            phase = (phase_msb << 2) | phase_lsb

            return {
                'amp': amp,
                'damp': damp,
                'ddamp': ddamp,
                'dddamp': dddamp,
                'phase': phase,
                'ftw': ftw,
                'chirp': chirp,
                'shift': shift
            }

        except struct.error as e:
            print(f"Error unpacking coefficient data: {e}")
            return None

    def read_status(self, dds_addr: int) -> Optional[Dict[str, Any]]:
        """
        Read current status from specified DDS.

        Args:
            dds_addr: DDS address (0-3)

        Returns:
            Dictionary with status information or None on error
        """
        if not (0 <= dds_addr <= 3):
            raise ValueError("DDS address must be 0-3")

        success = self._send_command(self.CMD_READ_STATUS, dds_addr)
        if not success:
            return None

        status, data = self._read_response()
        if status != self.RESP_DATA or not data:
            return None

        try:
            return {
                'target_addr': data[0] if len(data) > 0 else None,
                'last_command': data[1] if len(data) > 1 else None,
                'status_data': list(data[2:]) if len(data) > 2 else []
            }
        except Exception as e:
            print(f"Error parsing status data: {e}")
            return None

    def get_connection_info(self) -> Dict[str, Any]:
        """
        Get connection information.

        Returns:
            Dictionary with connection details
        """
        return {
            'port': self.port,
            'baud_rate': self.baud_rate,
            'timeout': self.timeout,
            'is_open': self.ser.is_open if self.ser else False
        }


# Example usage and test functions
def test_basic_communication(controller: LTC2000UARTController):
    """Test basic communication with all DDSs"""
    print("Testing basic communication...")

    for dds in range(4):
        print(f"  Pinging DDS {dds}...", end=" ")
        if controller.ping(dds):
            print("OK")
        else:
            print("FAILED")

def test_coefficient_write_read(controller: LTC2000UARTController):
    """Test writing and reading coefficients"""
    print("\nTesting coefficient write/read...")

    test_coeffs = {
        'amp': 1000,
        'damp': 500,
        'ddamp': 100,
        'dddamp': 10,
        'phase': 1024,
        'ftw': 1000000,
        'chirp': 1000,
        'shift': 2
    }

    dds_addr = 0
    print(f"  Writing coefficients to DDS {dds_addr}...")
    success = controller.write_coefficients(dds_addr, **test_coeffs)

    if success:
        print("  Write successful")

        print(f"  Reading coefficients from DDS {dds_addr}...")
        read_coeffs = controller.read_coefficients(dds_addr)

        if read_coeffs:
            print("  Read successful")
            print(f"  Original: {test_coeffs}")
            print(f"  Read back: {read_coeffs}")

            # Compare values
            all_match = True
            for key, orig_val in test_coeffs.items():
                read_val = read_coeffs.get(key)
                if read_val != orig_val:
                    print(f"    MISMATCH: {key} = {read_val}, expected {orig_val}")
                    all_match = False

            if all_match:
                print("  All coefficients match!")
            else:
                print("  Some coefficients don't match")
        else:
            print("  Read failed")
    else:
        print("  Write failed")

def test_multiple_dds_control(controller: LTC2000UARTController):
    """Test controlling multiple DDSs with different parameters"""
    print("\nTesting multiple DDS control...")

    base_ftw = 1000000
    base_amp = 2000

    for dds in range(4):
        print(f"  Configuring DDS {dds}...")
        success = controller.write_coefficients(
            dds_addr=dds,
            ftw=base_ftw + dds * 100000,  # Different frequency per DDS
            amp=base_amp + dds * 500,     # Different amplitude per DDS
            damp=100 + dds * 50,          # Different slope per DDS
            shift=dds                     # Different update rate per DDS
        )

        if success:
            print(f"    DDS {dds} configured successfully")
        else:
            print(f"    DDS {dds} configuration failed")


if __name__ == "__main__":
    # Example usage
    PORT = '/dev/ttyUSB0'  # Change to your serial port

    print(f"LTC2000 UART Controller Test")
    print(f"Port: {PORT}")
    print("=" * 50)

    try:
        with LTC2000UARTController(PORT) as controller:
            print(f"Connected to {PORT}")
            print(f"Connection info: {controller.get_connection_info()}")

            # Run tests
            test_basic_communication(controller)
            test_coefficient_write_read(controller)
            test_multiple_dds_control(controller)

            print("\nTests completed!")

    except ConnectionError as e:
        print(f"Connection error: {e}")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()