import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_jsp_gui = LaunchConfiguration('use_joint_state_publisher_gui')
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('omni_carver_description'))
    xacro_file = os.path.join(pkg_path,'description','omni_carver.urdf')
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
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params],
        name='joint_state_publisher_gui',
        namespace='omni_carver',    
        condition=IfCondition(use_jsp_gui)
    )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'), 
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='false',
            description='Use joint state publisher GUI if true'),
        robot_state_publisher_node,
        joint_state_publisher_gui_node
    ])