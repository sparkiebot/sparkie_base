from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch file argument declarations
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port to use'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial connection baud rate'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='1.0',
        description='Serial connection timeout in seconds'
    )
    
    # Main serial connection node
    serial_node = Node(
        package='sparkie_board_connect',
        executable='sparkie_board',
        name='sparkie_board',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout': LaunchConfiguration('timeout'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        timeout_arg,
        serial_node
    ])