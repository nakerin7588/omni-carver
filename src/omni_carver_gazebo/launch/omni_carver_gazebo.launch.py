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
    
    omni_carver_gazebo_path = os.path.join(
        get_package_share_directory('omni_carver_gazebo'))
    
    omni_carver_controller_path = os.path.join(
        get_package_share_directory('omni_carver_controller'))
    
    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(omni_carver_gazebo_path, 'worlds'), ':' +
            str(Path(omni_carver_description_path).parent.resolve())
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='default_world',
                          description='Gz sim World'),
           ]
    )

    robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    omni_carver_description_path, 'launch'), '/description.launch.py']),
                launch_arguments=[
                    ('use_sim_time', 'true'
                    )
                ]
             )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )

    xacro_file = os.path.join(omni_carver_description_path,
                              'gazebo_description',
                              'main.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')
    
    params = {'robot_description': robot_desc}
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        remappings=[('/joint_states', '/omni_carver/joint_states')],
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.01905',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'omni_carver',
                   '-b'],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],    
    )
    
    omni_node = Node(
        package='omni_carver_controller',
        executable='omni_drive_node_script.py',
        output='screen',
    )
    
    # Bridge
    bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(omni_carver_gazebo_path, 'config', 'gz_ros_bridge.yaml'),
    )

    rviz_config_file = os.path.join(omni_carver_gazebo_path, 'rviz', 'default_description.rviz')

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner, velocity_controller_spawner],
            )
        ),
        robot_description,
        gazebo_resource_path,
        arguments,
        gazebo,
        robot_state_publisher,
        gz_spawn_entity,
        bridge,
        omni_node,
        
    ])
