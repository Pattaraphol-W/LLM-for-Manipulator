import rospy
from std_msgs.msg import Float32, String
from keyence_plc_driver.ethernet_driver import PLCSocket, kvHostLink
import time

class PLCProtocol:
    """
    ROS node to communicate with a Keyence PLC via Ethernet using the HostLink protocol.
    This node handles both reading and writing to specified PLC addresses.
    """
    def __init__(self):
        rospy.init_node('plc_protocol_driver')

        # Get parameters from the launch file
        host = rospy.get_param('~host', '192.168.1.3')
        port = rospy.get_param('~port', 8501)
        
        # Read the list of devices to read from and write to
        self._read_devices = rospy.get_param('~read_devices', []).split(',')
        self._write_devices = rospy.get_param('~write_devices', []).split(',')
        
        # Rate for the main read loop
        self._rate = rospy.Rate(10)

        # Initialize the communication classes
        self._socket = PLCSocket(host=host, port=port)
        self._protocol = kvHostLink(self._socket)

        # Publishers for read devices
        self._publishers = {}
        for device in self._read_devices:
            topic_name = f"~read/{device.strip()}"
            self._publishers[device.strip()] = rospy.Publisher(topic_name, Float32, queue_size=10)

        # Subscriber for write commands
        self._write_sub = rospy.Subscriber(f"~write_command", String, self._handle_write_command)

        rospy.loginfo(f"PLC Driver initialized for host: {host}")
        rospy.loginfo(f"Read devices: {self._read_devices}")
        rospy.loginfo(f"Write devices: {self._write_devices}")

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
                rospy.logerr(f"PLC returned error code: {response_str}")
                return -1.0
            
            # Attempt to convert to float
            return float(response_str)
        except (UnicodeDecodeError, ValueError) as e:
            rospy.logerr(f"Failed to parse PLC response '{response_bytes}': {e}")
            return -1.0
        except Exception as e:
            rospy.logerr(f"Unexpected error parsing response: {e}")
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
                rospy.loginfo(f"Received write command for {address} with data {data}.")
                response_bytes = self._protocol.write(address, data)

                # Check the raw byte response for success
                if response_bytes and response_bytes.startswith(b'OK'):
                    rospy.loginfo(f"Successfully wrote {data} to {address}.")
                else:
                    rospy.logerr(f"Failed to write to {address}. Response: {response_bytes.decode('ascii', 'ignore') if response_bytes else 'None'}")
            else:
                rospy.logwarn(f"Received write command for unknown device: {address}")
        except Exception as e:
            rospy.logerr(f"Failed to process write command '{msg.data}': {e}")

    def spin(self):
        """Main loop to periodically read from the PLC and publish data."""
        if not self._socket.connect():
            rospy.logerr("Could not connect to PLC. Shutting down.")
            return

        while not rospy.is_shutdown():
            for device in self._read_devices:
                device = device.strip()
                response_bytes = self._protocol.read(device)
                
                # The main responsibility of this file is to parse the response
                parsed_value = self._parse_response(response_bytes)
                
                # Publish the parsed value
                if device in self._publishers:
                    self._publishers[device].publish(Float32(parsed_value))
                    rospy.loginfo(f"Published {parsed_value} for {device}")
                else:
                    rospy.logerr(f"No publisher found for device: {device}")

            self._rate.sleep()
        
        self._socket.disconnect()

if __name__ == '__main__':
    try:
        driver = PLCProtocol()
        driver.spin()
    except rospy.ROSInterruptException:
        pass

