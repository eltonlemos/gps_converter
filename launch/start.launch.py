from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    parameter = os.path.join(get_package_share_directory('gpsconv'), 'config', 'params.yaml')

    converter_node = Node(
        package='gpsconv',
        executable='gps_converter',
        name='gps_converter',
        output='screen',
        parameters=[parameter]
    )


    ld.add_action(converter_node)

    return ld