import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    omni_carver_description_path = os.path.join(
        get_package_share_directory('omni_carver_description'))
    
    omni_carver_controller_path = os.path.join(
        get_package_share_directory('omni_carver_controller'))

    # Launch the robot state publisher
    robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    omni_carver_description_path, 'launch'), '/description.launch.py']),
                launch_arguments=[
                    ('use_sim_time', 'false'
                    )
                ]
    )
    
    # Launch the ldlidar node
    ldlidar_launch = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('ldlidar_stl_ros2'),
          '/launch/ld06.launch.py'
      ])
    )
    
    #  Run the BNO055 IMU node
    bno055_node = Node(
        package='bno055_usb_stick', 
        executable='bno055_usb_stick_node_script.py',
        output='screen',
    )
    
    # Launch the omni drive node
    omni_node = Node(
        package='omni_carver_controller',
        executable='omni_drive_node_script.py',
        output='screen',
    )
    
    # Launch the arduino serial node
    arduino_serial_node = Node(
        package='omni_carver_arduino_serial',
        executable='arduino_serial_node_script.py',
        output='screen',
    )
    
    # Launch the robot localization node
    launch_robot_localization = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('omni_carver_localization'),
          '/launch/ekf.launch.py'
      ])
    )
    
    timer_action_1 = TimerAction(
        period=5.0,
        actions=[
            ldlidar_launch,
            bno055_node,
        ],
    )
    
    timer_action_2 = TimerAction(
        period=10.0,
        actions=[
            launch_robot_localization
        ],
    )
    
    return LaunchDescription([
        robot_description,
        omni_node,
        arduino_serial_node,
        timer_action_1,
        timer_action_2,
    ])
