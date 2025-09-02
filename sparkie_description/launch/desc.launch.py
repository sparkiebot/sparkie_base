import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sparkie_description').find('sparkie_description')
    default_model_path = os.path.join(pkg_share, 'urdf/sparkie_description.urdf')

    default_model_path = os.path.join(
        get_package_share_directory('sparkie_description'),
        'urdf',
        'sparkiebot.xacro'
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        robot_state_publisher_node
    ])
