import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    # Define filenames
    description_package_name = "omni_carver_description"
    gazebo_package_name = "omni_carver_gazebo"
    rviz_file_name = "gazebo_rviz_config.rviz"
    
    # Get path of important files
    gazebo_pacakge_path = get_package_share_directory(gazebo_package_name)
    rviz_file_path = os.path.join(
        get_package_share_directory(gazebo_package_name),
        'rviz',
        rviz_file_name
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(description_package_name),
                    "launch",
                    "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch", "gz_sim.launch.py"
                )
            ]
        ),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([gazebo_pacakge_path, 'worlds/my_world.sdf'])],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="gz_spawn_model.launch.py",
        arguments=[
            "-topic", "robot_description",
            "entity_name:=", "my_bot"
        ],
        output = "screen"
    )
    
    controller = Node(
    	package="my_controller",
    	executable="diff_drive.py"
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file_path
        ],
        output = "screen"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
    )

    # Add the rest of the nodes and launch descriptions
    launch_description.add_action(rviz)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    # launch_description.add_action(controller)
    launch_description.add_action(rsp)
    return launch_description