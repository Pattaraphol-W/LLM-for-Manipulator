from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare Arguments (Replaces <arg ... />)
    plc_host_arg = DeclareLaunchArgument(
        'plc_host',
        default_value='192.168.1.10',
        description='IP address of the Keyence PLC'
    )

    plc_port_arg = DeclareLaunchArgument(
        'plc_port',
        default_value='8501',
        description='Port of the Keyence PLC'
    )

    # 2. Define Nodes (Replaces <node ... />)
    
    # Joystick Node
    joy_node = Node(
        package='joy',
        executable='joy_node', # In ROS 2, 'type' is renamed to 'executable'
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05
        }]
    )

    # Unified PLC Connection Protocol Driver
    plc_driver_node = Node(
        package='keyence_plc_driver',
        executable='PLCconncectionProtocol.py', # Must match the exact script name
        name='plc_ethernet_driver',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('plc_host'),
            'port': LaunchConfiguration('plc_port'),
            'read_devices': 'R603,R604,R605,MR012,MR013,MR100,MR101,MR102,MR103,MR104,MR105,MR106,MR107,MR206,MR207,MR208,MR209,MR400,MR401,MR402,MR403,MR404,MR405,MR406,MR602,MR603,MR604,MR605,MR606',
            'write_devices': 'R206,MR206,MR207,MR208,MR209,MR210,MR211,MR214,MR400,MR401,MR402,MR403,MR404,MR405,MR406,MR602,MR603,MR604,MR605,MR606'
        }]
    )

    # Monitor GUI Node
    agv_control_node = Node(
        package='keyence_plc_driver',
        executable='AGV_control.py',
        name='AGV_control',
        output='screen'
    )

    # Teaching Node
    robot_control_node = Node(
        package='keyence_plc_driver',
        executable='robot_control.py',
        name='Robot_control',
        output='screen'
    )

    # 3. Return the LaunchDescription object
    return LaunchDescription([
        plc_host_arg,
        plc_port_arg,
        joy_node,
        plc_driver_node,
        agv_control_node,
        robot_control_node
    ])
