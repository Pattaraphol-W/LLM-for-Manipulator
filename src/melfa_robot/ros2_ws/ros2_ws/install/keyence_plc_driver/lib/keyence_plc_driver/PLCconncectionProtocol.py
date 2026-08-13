#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from keyence_plc_ethernet_driver.ethernet_driver import PLCSocket, kvHostLink

class PLCProtocol(Node):
    """
    ROS2 node to communicate with a Keyence PLC via Ethernet using the HostLink protocol.
    This node handles both reading and writing to specified PLC addresses.
    """
    def __init__(self):
        super().__init__('plc_protocol_driver')

        # Get parameters from the launch file
        self.declare_parameter('host', '192.168.1.3')
        self.declare_parameter('port', 8501)
        self.declare_parameter('read_devices', '')
        self.declare_parameter('write_devices', '')

        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        # Read the list of devices to read from and write to
        self._read_devices = self.get_parameter('read_devices').get_parameter_value().string_value.split(',')
        self._write_devices = self.get_parameter('write_devices').get_parameter_value().string_value.split(',')

        # Initialize the communication classes
        self._socket = PLCSocket(host=host, port=port)
        self._protocol = kvHostLink(self._socket)

        # Publishers for read devices
        self._plc_publishers = {}
        for device in self._read_devices:
            topic_name = f"~/read/{device.strip()}"
            self._plc_publishers[device.strip()] = self.create_publisher(Float32, topic_name, 10)

        # Subscriber for write commands
        self._plc_write_sub = self.create_subscription(
            String,
            '/write_command',
            self._handle_write_command,
            10
        )

        # Timer for main read loop at 20Hz
        self._plc_timer = self.create_timer(1.0 / 20.0, self._read_loop)

        self.get_logger().info(f"PLC Driver initialized for host: {host}")
        self.get_logger().info(f"Read devices: {self._read_devices}")
        self.get_logger().info(f"Write devices: {self._write_devices}")

        # Connect to PLC
        if not self._socket.connect():
            self.get_logger().error("Could not connect to PLC. Shutting down.")
            raise RuntimeError("PLC connection failed")

    def _parse_response(self, response_bytes):
        """
        Parses a raw byte response from the PLC and returns a float value.
        Returns a float, or -1.0 on error.
        """
        if not response_bytes:
            return -1.0
        
        try:
            # Strip the carriage return and any leading/trailing whitespace
            response_str = response_bytes.decode('utf-8').strip()
            
            # Check for common PLC error codes
            if response_str.startswith('E'):
                self.get_logger().error(f"PLC returned error code: {response_str}")
                return -1.0
            
            # Attempt to convert to float
            return float(response_str)
        except (UnicodeDecodeError, ValueError) as e:
            self.get_logger().error(f"Failed to parse PLC response '{response_bytes}': {e}")
            return -1.0
        except Exception as e:
            self.get_logger().error(f"Unexpected error parsing response: {e}")
            return -1.0

    def _handle_write_command(self, msg):
        """
        Callback for the write command subscriber.
        Expects a message of format "ADDRESS,DATA".
        """
        try:
            parts = msg.data.split(',')
            address = parts[0].strip()
            data = parts[1].strip()

            if address in self._write_devices:
                self.get_logger().info(f"Received write command for {address} with data {data}.")
                response_bytes = self._protocol.write(address, data)

                # Check the raw byte response for success
                if response_bytes and response_bytes.startswith(b'OK'):
                    self.get_logger().info(f"Successfully wrote {data} to {address}.")
                else:
                    self.get_logger().error(
                        f"Failed to write to {address}. Response: {response_bytes.decode('ascii', 'ignore') if response_bytes else 'None'}"
                    )
            else:
                self.get_logger().warn(f"Received write command for unknown device: {address}")
        except Exception as e:
            self.get_logger().error(f"Failed to process write command '{msg.data}': {e}")

    def _read_loop(self):
        """Timer callback to periodically read from the PLC and publish data."""
        for device in self._read_devices:
            device = device.strip()
            response_bytes = self._protocol.read(device)
            
            # The main responsibility of this file is to parse the response
            parsed_value = self._parse_response(response_bytes)
            
            # Publish the parsed value
            if device in self._plc_publishers:
                self._plc_publishers[device].publish(Float32(data=parsed_value))
            else:
                self.get_logger().error(f"No publisher found for device: {device}")

    def destroy_node(self):
        self._socket.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        driver = PLCProtocol()
        rclpy.spin(driver)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
