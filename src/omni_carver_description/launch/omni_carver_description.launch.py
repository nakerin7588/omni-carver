import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    pkg_path = os.path.join(get_package_share_directory('omni_carver_description'))
    
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [
                os.path.join(
                    pkg_path,
                    "launch",
                    "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"false"}.items()
    )
    
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'default_description.rviz')]
    )
    
    ld = LaunchDescription()
    ld.add_action(rsp)
    ld.add_action(jsp)
    ld.add_action(rviz)
    
    return ld