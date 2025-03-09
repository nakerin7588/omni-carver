import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():

    omni_carver_description_path = os.path.join(
        get_package_share_directory('omni_carver_description'))
    
    omni_carver_controller_path = os.path.join(
        get_package_share_directory('omni_carver_controller'))

    robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    omni_carver_description_path, 'launch'), '/description.launch.py']),
                launch_arguments=[
                    ('use_sim_time', 'false'
                    )
                ]
    )
    
    omni_node = Node(
        package='omni_carver_controller',
        executable='omni_drive_node_script.py',
        output='screen',
    )

    return LaunchDescription([
        robot_description,
        omni_node,
    ])
