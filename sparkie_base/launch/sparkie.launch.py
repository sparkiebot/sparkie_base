from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch file argument declarations
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for ESP32 board connection'
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
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('sparkie_description'),
            'urdf',
            'sparkiebot.xacro'
        ]),
        description='Path to robot URDF/Xacro file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    # Include robot description launch file
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sparkie_description'),
                'launch',
                'desc.launch.py'
            ])
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model')
        }.items()
    )
    
    # Include board connection launch file
    board_connection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sparkie_board_connect'),
                'launch',
                'board.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout': LaunchConfiguration('timeout')
        }.items()
    )

    use_perception_arg = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description='Enable perception subsystem (LIDAR, odometry, IMU)'
    )

    # Include perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sparkie_perception'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_perception'))
    )
    
    # Optional RViz2 launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sparkie_base'),
                'launch',
                'rviz.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    
    
    return LaunchDescription([
        # Arguments
        serial_port_arg,
        baud_rate_arg,
        timeout_arg,
        model_arg,
        use_rviz_arg,
        use_perception_arg,

        # Launch files
        robot_description_launch,
        board_connection_launch,
        perception_launch,

        # RViz
        rviz_launch,
    ])
