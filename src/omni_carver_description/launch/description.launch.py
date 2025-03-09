import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('omni_carver_description'))
    # xacro_file = os.path.join(pkg_path,'description','omni_carver_core.urdf.xacro')
    xacro_file = os.path.join(pkg_path,'gazebo_description','main.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        namespace='omni_carver',
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params],
        name='joint_state_publisher',
        namespace='omni_carver',    
        # remappings=[('robot_description', '/omni_carver/robot_description')]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params],
        name='joint_state_publisher_gui',
        namespace='omni_carver',    
    )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'), 
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node
        
    ])