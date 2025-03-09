import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('omni_carver_description'))
    
    controller_node = Node(
        package='omni_drive_controller',
        executable='omni_drive_node',
        output='screen',
        parameters=[{'wheel_radius': 0.038, 'wheel_base': 0.04, 'rate': 100}],
    )
    
    # Launch!
    return LaunchDescription([
        controller_node,
    ])