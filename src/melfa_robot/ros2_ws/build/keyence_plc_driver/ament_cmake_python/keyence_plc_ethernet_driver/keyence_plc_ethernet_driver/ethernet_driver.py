#!/usr/bin/env python3
import socket
import logging

# Use standard Python logging instead of rospy — works with any ROS2 node or standalone
logger = logging.getLogger(__name__)

class PLCSocket:
    """Handles the low-level socket communication using TCP (SOCK_STREAM)."""
    def __init__(self, host, port=8501):
        self.host = host
        self.port = port
        self.addr = (host, port)
        self._client = None
        self._connected = False

    def connect(self):
        """Creates and connects the TCP socket."""
        try:
            self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._client.settimeout(2.0)
            self._client.connect(self.addr)
            self._connected = True
            logger.info(f"PLCSocket: Successfully connected to {self.host}:{self.port}.")
            return True
        except Exception as e:
            logger.error(f"PLCSocket: Failed to connect - {e}")
            self._connected = False
            return False

    def send_receive(self, command_bytes):
        """Sends bytes and waits for a response from the PLC."""
        if not self._connected:
            logger.warning("PLCSocket: Not connected.")
            return None
        try:
            self._client.sendall(command_bytes)
            response_raw = self._client.recv(1024)
            return response_raw
        except socket.timeout:
            logger.warning("PLCSocket: Socket operation timed out.")
            return None
        except socket.error as e:
            logger.error(f"PLCSocket: Socket error during send/recv - {e}")
            self._connected = False
            return None

    def disconnect(self):
        """Closes the socket connection."""
        if self._client:
            self._client.close()
        self._connected = False
        logger.info("PLCSocket: Disconnected.")


class kvHostLink:
    """Manages the Keyence KV-HostLink protocol for reading and writing."""
    def __init__(self, plc_socket):
        self._socket = plc_socket

    def read(self, address_suffix):
        """
        Formats a read command and sends it.
        Returns the raw byte response.
        """
        command_string = f"RD {address_suffix}"
        response_bytes = self._socket.send_receive(command_string.encode('utf-8') + b'\r')
        return response_bytes

    def write(self, address_suffix, data):
        """
        Formats a write command and sends it.
        Returns the raw byte response.
        """
        command_string = f"WR {address_suffix} {data}"
        response_bytes = self._socket.send_receive(command_string.encode('utf-8') + b'\r')
        return response_bytes
