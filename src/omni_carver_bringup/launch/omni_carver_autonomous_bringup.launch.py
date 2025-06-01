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
    # Launch the ldlidar node
    main_bringup = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('omni_carver_bringup'),
          '/launch/omni_carver_bringup.launch.py'
      ])
    )
    # Launch the robot localization node
    nav_bringup = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('omni_carver_nav2'),
          '/launch/omni_carver_nav2_bringup.launch.py'
      ])
    )
    
    scheduler = Node(
        package='omni_carver_scheduler',
        executable='scheduler_node_script.py',
        output='screen',
    )
    
    initial_pose_pub = ExecuteProcess(
    cmd=[
        'ros2', 'topic', 'pub', '-1', '/initialpose',
        'geometry_msgs/msg/PoseWithCovarianceStamped',
        '{ header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },'
        '  pose: { pose: { position: { x: 1.0, y: 2.0, z: 0.0 },'
        '                     orientation: { z: 0.0, w: 1.0 } },'
        '           covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,'
        '                        0,0,0,0,0,0, 0,0,0,0,0,0,'
        '                        0,0,0,0,0,0.07] } }'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        main_bringup,
        TimerAction(
            period=10.0,
            actions=[
                nav_bringup,
                scheduler,
            ]
        ),
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         initial_pose_pub,
        #     ]
        # ),
    ])