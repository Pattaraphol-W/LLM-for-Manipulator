import socket
import binascii
import time
import random
import string

# --- Configuration ---
PLC_HOST = '192.168.1.3'
PLC_PORT = 5000
TIMEOUT_SECONDS = 2.0

# --- Checksum Calculation Functions ---
def calculate_keyence_checksum_xor_all(data_bytes):
    """
    Calculates the XOR checksum for Keyence PLC commands.
    This is typically the XOR sum of all bytes from STX (0x02) up to, but not including, ETX (0x03).
    """
    checksum = 0
    for byte_val in data_bytes:
        checksum ^= byte_val
    return checksum

def calculate_crc16_modbus(data_bytes):
    """
    Calculates CRC-16 (Modbus variant) checksum.
    Polynomial: 0xA001 (or 0x8005 reversed)
    Initial value: 0xFFFF
    XOR out: 0x0000
    Reverse in: True
    Reverse out: True
    """
    crc = 0xFFFF
    poly = 0xA001 # Standard CRC-16-Modbus polynomial (reversed)

    for byte_val in data_bytes:
        crc ^= byte_val
        for _ in range(8):
            if (crc & 0x0001):
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    # Ensure CRC is within 16 bits
    crc &= 0xFFFF
    return crc

def calculate_crc16_ccitt(data_bytes):
    """
    Calculates CRC-16-CCITT checksum.
    Polynomial: 0x1021
    Initial value: 0x0000 (or 0xFFFF, 0x1D0F depending on variant)
    Let's use 0x0000 for this test.
    """
    crc = 0x0000
    poly = 0x1021

    for byte_val in data_bytes:
        crc ^= (byte_val << 8)
        for _ in range(8):
            if (crc & 0x8000):
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
    crc &= 0xFFFF
    return crc

def calculate_crc16_ibm(data_bytes):
    """
    Calculates CRC-16-IBM checksum.
    Polynomial: 0x8005
    Initial value: 0x0000
    XOR out: 0x0000
    Reverse in: True
    Reverse out: True
    """
    crc = 0x0000
    poly = 0x8005 # Standard CRC-16-IBM polynomial

    for byte_val in data_bytes:
        crc ^= byte_val
        for _ in range(8):
            if (crc & 0x0001):
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    crc &= 0xFFFF
    return crc


# --- Helper function to create Keyence-formatted command bytes ---
def create_keyence_command(
    command_body_str=None,
    raw_command_body_bytes=None,
    unit_number=None,
    station_number=None,
    checksum_type='xor',      # 'xor', 'crc16_modbus', 'crc16_ccitt', 'crc16_ibm', 'none'
    send_checksum_as_raw_bytes=False,
    checksum_byte_order='big',
    start_bytes=b'\x02',      # Fixed to 0x02 for current tests
    include_etx=True,
    final_terminator_bytes=b'\x0D',
    pad_command_body_to=None,
    unit_number_format='2hex',
    station_number_format='2hex',
    include_data_length=False,
    data_length_format='2hex',
    data_length_byte_order='big',
    data_length_scope='body_etx_checksum', # 'body_etx_checksum', 'body_only', 'body_device_only', 'unit_body_etx_checksum', 'unit_body_only'
    sequence_number=None,
    sequence_number_format='2hex',
    fixed_header_bytes=b'', # Optional fixed header bytes after STX (and before Unit/Length)
    fixed_frame_length=None # Pad the entire frame to a fixed length
):
    """
    Creates Keyence PLC command bytes with various formatting options.
    """
    if raw_command_body_bytes is not None:
        command_body_bytes = raw_command_body_bytes
        command_body_str_for_desc = raw_command_body_bytes.hex() # For description
    elif command_body_str is not None:
        if pad_command_body_to:
            command_body_str = command_body_str.ljust(pad_command_body_to)
        command_body_bytes = command_body_str.encode('ascii')
        command_body_str_for_desc = command_body_str
    else:
        raise ValueError("Either command_body_str or raw_command_body_bytes must be provided.")


    # Build the frame for checksum calculation (if checksum is included)
    frame_parts_for_checksum = []
    if start_bytes:
        frame_parts_for_checksum.append(start_bytes)

    if fixed_header_bytes:
        frame_parts_for_checksum.append(fixed_header_bytes)

    unit_bytes_for_frame = None
    station_bytes_for_frame = None
    data_length_bytes = None
    sequence_bytes_for_frame = None

    if unit_number is not None:
        if unit_number_format == '2hex':
            unit_bytes_for_frame = f"{unit_number:02X}".encode('ascii')
        elif unit_number_format == '1ascii':
            unit_bytes_for_frame = str(unit_number).encode('ascii')
        else:
            raise ValueError("Invalid unit_number_format")
        frame_parts_for_checksum.append(unit_bytes_for_frame)

    if station_number is not None:
        if station_number_format == '2hex':
            station_bytes_for_frame = f"{station_number:02X}".encode('ascii')
        elif station_number_format == '1ascii':
            station_bytes_for_frame = str(station_number).encode('ascii')
        else:
            raise ValueError("Invalid station_number_format")
        frame_parts_for_checksum.append(station_bytes_for_frame)

    # Add Sequence Number
    if sequence_number is not None:
        if sequence_number_format == '2hex':
            sequence_bytes_for_frame = f"{sequence_number:02X}".encode('ascii')
        elif sequence_number_format == '1ascii':
            sequence_bytes_for_frame = str(sequence_number).encode('ascii')
        elif sequence_number_format == 'raw_byte':
            sequence_bytes_for_frame = sequence_number.to_bytes(1, byteorder='big')
        else:
            raise ValueError("Invalid sequence_number_format")
        frame_parts_for_checksum.append(sequence_bytes_for_frame)

    # Calculate data length if required
    if include_data_length:
        data_len = 0
        if data_length_scope == 'body_etx_checksum':
            data_len = len(command_body_bytes)
            if include_etx:
                data_len += 1
            if checksum_type != 'none':
                if checksum_type in ['crc16_modbus', 'crc16_ccitt', 'crc16_ibm']:
                    data_len += (2 if send_checksum_as_raw_bytes else 4)
                elif checksum_type == 'xor':
                    data_len += (1 if send_checksum_as_raw_bytes else 2)
        elif data_length_scope == 'body_only':
            data_len = len(command_body_bytes)
        elif data_length_scope == 'body_device_only':
            if command_body_str:
                parts = command_body_str.split(' ', 1)
                if len(parts) > 1:
                    data_len = len(parts[1].encode('ascii'))
                else:
                    data_len = len(command_body_bytes)
            else:
                data_len = len(command_body_bytes)
        elif data_length_scope == 'unit_body_etx_checksum':
            # Ensure unit_bytes_for_frame is not None for calculation
            current_unit_bytes = unit_bytes_for_frame if unit_bytes_for_frame is not None else b''
            data_len = len(current_unit_bytes) + len(command_body_bytes)
            if include_etx:
                data_len += 1
            if checksum_type != 'none':
                if checksum_type in ['crc16_modbus', 'crc16_ccitt', 'crc16_ibm']:
                    data_len += (2 if send_checksum_as_raw_bytes else 4)
                elif checksum_type == 'xor':
                    data_len += (1 if send_checksum_as_raw_bytes else 2)
        elif data_length_scope == 'unit_body_only':
            # Ensure unit_bytes_for_frame is not None for calculation
            current_unit_bytes = unit_bytes_for_frame if unit_bytes_for_frame is not None else b''
            data_len = len(current_unit_bytes) + len(command_body_bytes)


        if data_length_format == '2hex':
            data_length_bytes = f"{data_len:02X}".encode('ascii')
        elif data_length_format == '4hex':
            data_length_bytes = f"{data_len:04X}".encode('ascii')
        elif data_length_format == 'raw_byte':
            data_length_bytes = data_len.to_bytes(1, byteorder=data_length_byte_order)
        elif data_length_format == 'raw_word':
            data_length_bytes = data_len.to_bytes(2, byteorder=data_length_byte_order)
        else:
            raise ValueError("Invalid data_length_format")
        frame_parts_for_checksum.append(data_length_bytes)


    frame_parts_for_checksum.append(command_body_bytes)

    if include_etx:
        frame_parts_for_checksum.append(b'\x03') # ETX

    frame_for_checksum = b''.join(frame_parts_for_checksum)

    # Calculate Checksum (if not omitted)
    checksum_bytes = b''
    checksum_display_str = "N/A"
    if checksum_type != 'none':
        if checksum_type == 'crc16_modbus':
            checksum_val = calculate_crc16_modbus(frame_for_checksum)
            if send_checksum_as_raw_bytes:
                checksum_bytes = checksum_val.to_bytes(2, byteorder=checksum_byte_order)
                checksum_display_str = f"raw({checksum_bytes.hex()})"
            else:
                checksum_bytes = f"{checksum_val:04X}".encode('ascii')
                checksum_display_str = f"{checksum_val:04X}"
        elif checksum_type == 'crc16_ccitt':
            checksum_val = calculate_crc16_ccitt(frame_for_checksum)
            if send_checksum_as_raw_bytes:
                checksum_bytes = checksum_val.to_bytes(2, byteorder=checksum_byte_order)
                checksum_display_str = f"raw({checksum_bytes.hex()})"
            else:
                checksum_bytes = f"{checksum_val:04X}".encode('ascii')
                checksum_display_str = f"{checksum_val:04X}"
        elif checksum_type == 'crc16_ibm':
            checksum_val = calculate_crc16_ibm(frame_for_checksum)
            if send_checksum_as_raw_bytes:
                checksum_bytes = checksum_val.to_bytes(2, byteorder=checksum_byte_order)
                checksum_display_str = f"raw({checksum_bytes.hex()})"
            else:
                checksum_bytes = f"{checksum_val:04X}".encode('ascii')
                checksum_display_str = f"{checksum_val:04X}"
        elif checksum_type == 'xor':
            checksum_val = calculate_keyence_checksum_xor_all(frame_for_checksum)
            if send_checksum_as_raw_bytes:
                checksum_bytes = checksum_val.to_bytes(1, byteorder=checksum_byte_order)
                checksum_display_str = f"raw({checksum_bytes.hex()})"
            else:
                checksum_bytes = f"{checksum_val:02X}".encode('ascii')
                checksum_display_str = f"{checksum_val:02X}"
        else:
            raise ValueError("Invalid checksum_type")


    # Final command bytes assembly
    final_command_bytes_parts = []
    if start_bytes:
        final_command_bytes_parts.append(start_bytes)

    if fixed_header_bytes:
        final_command_bytes_parts.append(fixed_header_bytes)

    if unit_number is not None:
        final_command_bytes_parts.append(unit_bytes_for_frame)
    if station_number is not None:
        final_command_bytes_parts.append(station_bytes_for_frame)
    if sequence_number is not None:
        final_command_bytes_parts.append(sequence_bytes_for_frame)
    if include_data_length:
        final_command_bytes_parts.append(data_length_bytes)

    final_command_bytes_parts.append(command_body_bytes)

    if include_etx:
        final_command_bytes_parts.append(b'\x03') # ETX

    if checksum_type != 'none':
        final_command_bytes_parts.append(checksum_bytes)
    final_command_bytes_parts.append(final_terminator_bytes)

    final_command_bytes = b''.join(final_command_bytes_parts)

    # Apply fixed_frame_length padding
    if fixed_frame_length is not None:
        if len(final_command_bytes) < fixed_frame_length:
            final_command_bytes = final_command_bytes.ljust(fixed_frame_length, b'\x00') # Pad with NULL bytes
        elif len(final_command_bytes) > fixed_frame_length:
            print(f"Warning: Command length {len(final_command_bytes)} exceeds fixed_frame_length {fixed_frame_length}. Truncating.")
            final_command_bytes = final_command_bytes[:fixed_frame_length]


    # Create descriptive string
    description_parts = []
    if start_bytes:
        description_parts.append(f"Start({start_bytes.hex()})")
    else:
        description_parts.append("NoStartByte")
    if fixed_header_bytes:
        description_parts.append(f"FixedHeader({fixed_header_bytes.hex()})")
    if unit_bytes_for_frame is not None:
        description_parts.append(f"Unit({unit_bytes_for_frame.decode('ascii')})")
    if station_bytes_for_frame is not None:
        description_parts.append(f"Station({station_bytes_for_frame.decode('ascii')})")
    if sequence_bytes_for_frame is not None:
        description_parts.append(f"Seq({sequence_bytes_for_frame.decode('ascii') if sequence_number_format != 'raw_byte' else sequence_bytes_for_frame.hex()})")
    if include_data_length:
        if data_length_format in ['raw_byte', 'raw_word']:
            description_parts.append(f"Length(raw:{data_length_bytes.hex()})")
        else:
            description_parts.append(f"Length({data_length_bytes.decode('ascii')})")
        description_parts.append(f"LengthScope({data_length_scope})")
    description_parts.append(f"'{command_body_str_for_desc}'")
    if include_etx:
        description_parts.append("ETX")
    if checksum_type != 'none':
        description_parts.append(f"Checksum({checksum_display_str})")
    description_parts.append(f"Terminator({final_terminator_bytes.hex()})")
    if checksum_type == 'crc16_modbus':
        description_parts.append("(CRC16 Modbus)")
    elif checksum_type == 'crc16_ccitt':
        description_parts.append("(CRC16 CCITT)")
    elif checksum_type == 'crc16_ibm':
        description_parts.append("(CRC16 IBM)")
    if send_checksum_as_raw_bytes:
        description_parts.append(f"(Raw Checksum {checksum_byte_order})")
    if pad_command_body_to:
        description_parts.append(f"(Padded to {pad_command_body_to})")
    if raw_command_body_bytes is not None:
        description_parts.append("(Raw Command Body)")
    if fixed_frame_length is not None:
        description_parts.append(f"(FixedFrameLen:{fixed_frame_length})")


    return final_command_bytes, " + ".join(description_parts)

# --- List of Commands to Test ---
COMMANDS_TO_TEST = []

# --- Generate a large number of random commands ---
NUM_RANDOM_COMMANDS = 20000 # Increased to 1000

for i in range(NUM_RANDOM_COMMANDS):
    # Random command body
    cmd_len = random.randint(3, 15)
    cmd_body = ''.join(random.choice(string.ascii_uppercase + string.digits + ' .') for _ in range(cmd_len))

    # Random parameters
    unit_num = random.choice([0, 1, 10, None])
    station_num = random.choice([0, 1, None]) if unit_num is not None else None # Station usually with unit
    seq_num = random.choice([0, 1, 2, 5, 10, None])
    include_dl = random.choice([True, False])
    checksum = random.choice(['xor', 'crc16_modbus', 'crc16_ccitt', 'crc16_ibm', 'none'])
    raw_checksum = random.choice([True, False]) if checksum != 'none' else False
    cs_byte_order = random.choice(['little', 'big'])
    include_etx_val = random.choice([True, False])
    term_bytes = random.choice([b'\x0D', b'\x0A', b'\r\n', b'\x00'])
    pad_body = random.choice([None, 5, 10, 15]) # Some padding
    dl_format = random.choice(['2hex', '4hex', 'raw_byte', 'raw_word'])
    dl_byte_order = random.choice(['little', 'big'])
    dl_scope = random.choice(['body_etx_checksum', 'body_only', 'unit_body_etx_checksum', 'unit_body_only'])
    fixed_hdr = random.choice([b'', b'\x00', b'\x00\x00', b'ABC', b'\x10\x20'])
    fixed_len = random.choice([None, 20, 32, 64])

    # Sometimes use raw binary command body
    if random.random() < 0.1: # 10% chance for raw binary
        raw_cmd_len = random.randint(2, 10)
        raw_cmd_body = bytes([random.randint(0, 255) for _ in range(raw_cmd_len)])
        cmd_body = None # Ensure command_body_str is None when raw_command_body_bytes is used
    else:
        cmd_body = ''.join(random.choice(string.ascii_uppercase + string.digits + ' .') for _ in range(cmd_len))
        raw_cmd_body = None

    try:
        cmd, desc = create_keyence_command(
            command_body_str=cmd_body,
            raw_command_body_bytes=raw_cmd_body,
            unit_number=unit_num,
            station_number=station_num,
            checksum_type=checksum,
            send_checksum_as_raw_bytes=raw_checksum,
            checksum_byte_order=cs_byte_order,
            include_etx=include_etx_val,
            final_terminator_bytes=term_bytes,
            pad_command_body_to=pad_body,
            unit_number_format=random.choice(['2hex', '1ascii']),
            station_number_format=random.choice(['2hex', '1ascii']),
            include_data_length=include_dl,
            data_length_format=dl_format,
            data_length_byte_order=dl_byte_order,
            data_length_scope=dl_scope,
            sequence_number=seq_num,
            sequence_number_format=random.choice(['2hex', '1ascii', 'raw_byte']),
            fixed_header_bytes=fixed_hdr,
            fixed_frame_length=fixed_len
        )
        COMMANDS_TO_TEST.append((cmd, desc))
    except ValueError as e:
        print(f"Skipping command generation due to error: {e}")
        continue


def get_raw_plc_response(host, port, command_bytes, command_description, timeout=2.0):
    """
    Connects to the PLC, sends a command, and returns the raw byte response.
    """
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)

        print(f"\n--- Attempting to connect to {host}:{port} for command: '{command_description}' ---")
        sock.connect((host, port))
        print(f"Successfully connected to {host}:{port}.")

        print(f"Sending command (raw bytes): {command_bytes}")
        sock.sendall(command_bytes)

        raw_response = sock.recv(1024)

        if not raw_response:
            print("Received empty response from PLC.")
            return None
        else:
            print(f"\n--- Raw Response Received ---")
            print(f"Bytes: {raw_response}")
            print(f"Hex:   {binascii.hexlify(raw_response).decode('ascii')}")
            print(f"Length: {len(raw_response)} bytes")
            print(f"-----------------------------")
            return raw_response

    except socket.timeout:
        print(f"Connection/Receive timed out after {timeout} seconds.")
    except ConnectionRefusedError:
        print(f"Connection refused by PLC at {host}:{port}. (Is the PLC program listening?)")
    except socket.error as e:
        print(f"Socket error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if sock:
            sock.close()
    return None

# --- Run the script ---
if __name__ == "__main__":
    for command_bytes, command_description in COMMANDS_TO_TEST:
        raw_data = get_raw_plc_response(PLC_HOST, PLC_PORT, command_bytes, command_description, TIMEOUT_SECONDS)

        if raw_data and raw_data != b'\x82P': # Stop if response is not 8250
            print(f"\n!!! SUCCESS !!! Received a different response: {raw_data.hex()}")
            break # Exit the loop upon successful response

        if raw_data:
            print("\n--- Decoding Attempts ---")
            # Try common decodings for the response
            try:
                print(f"UTF-8:   '{raw_data.decode('utf-8')}' (May still fail if not UTF-8)")
            except UnicodeDecodeError:
                print(f"UTF-8:   <Decoding failed - raw bytes not UTF-8>")

            try:
                print(f"Latin-1: '{raw_data.decode('latin-1')}'")
            except UnicodeDecodeError:
                print(f"Latin-1: <Decoding failed unexpectedly>")

            try:
                print(f"Shift-JIS: '{raw_data.decode('shift_jis')}'")
            except UnicodeDecodeError:
                print(f"Shift-JIS: <Decoding failed - raw bytes not Shift-JIS>")
            print("-------------------------")
        else:
            print("No raw data to decode for this command.")

        time.sleep(0.05) # Reduced delay for faster testing

