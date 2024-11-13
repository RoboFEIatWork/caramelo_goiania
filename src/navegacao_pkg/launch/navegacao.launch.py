from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    serial_ros_arduino_node = Node(
        package='navegacao_pkg',
        executable='serial_node_arduino',
        name='serial_node_arduino',
        output='screen'
    )

    cinematica_node = Node(
        package='navegacao_pkg',
        executable='cinematica',
        name='cinematica',
        output='screen'
    )

    return LaunchDescription([
        serial_ros_arduino_node,
        cinematica_node,
    ])