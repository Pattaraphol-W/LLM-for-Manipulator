import socket
import binascii # To convert raw bytes to hex for easy viewing

# --- Configuration ---
PLC_HOST = '192.168.1.3'  # Your PLC's IP address
PLC_PORT = 5000          # Your PLC's port
COMMAND_TO_SEND = 'RD R01' # Command to send (e.g., RD R01, RD DM0000.U)
TIMEOUT_SECONDS = 2.0    # How long to wait for a response

def get_raw_plc_response(host, port, command, timeout=2.0):
    """
    Connects to the PLC, sends a command, and returns the raw byte response.
    """
    sock = None
    try:
        # Create a TCP socket (SOCK_STREAM)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout) # Set a timeout for all socket operations

        print(f"Attempting to connect to {host}:{port}...")
        sock.connect((host, port))
        print(f"Successfully connected to {host}:{port}.")

        # Keyence commands usually need a Carriage Return (CR) terminator
        # And are typically ASCII encoded
        full_command = (command + '\r').encode('ascii')
        print(f"Sending command (ASCII bytes): {full_command}")

        sock.sendall(full_command) # Send the command

        # Receive the response
        # Adjust buffer size if you expect very long responses, but 1024 is usually fine
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
        print(f"Connection refused by PLC at {host}:{port}. Is the PLC program listening?")
    except socket.error as e:
        print(f"Socket error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if sock:
            sock.close()
            print("Socket closed.")
    return None

# --- Run the script ---
if __name__ == "__main__":
    raw_data = get_raw_plc_response(PLC_HOST, PLC_PORT, COMMAND_TO_SEND, TIMEOUT_SECONDS)

    if raw_data:
        # If we got raw data, we can try some common decodings just for display
        print("\n--- Decoding Attempts ---")
        try:
            print(f"UTF-8:   '{raw_data.decode('utf-8')}' (May fail with your byte 0xd2)")
        except UnicodeDecodeError:
            print(f"UTF-8:   <Decoding failed - raw bytes not UTF-8>")

        try:
            print(f"Latin-1: '{raw_data.decode('latin-1')}'")
        except UnicodeDecodeError: # Latin-1 should not fail on raw bytes
            print(f"Latin-1: <Decoding failed unexpectedly>")

        try:
            print(f"Shift-JIS: '{raw_data.decode('shift_jis')}'")
        except UnicodeDecodeError:
            print(f"Shift-JIS: <Decoding failed - raw bytes not Shift-JIS>")
        print("-------------------------")

    else:
        print("No raw data to decode.")

