import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # กำหนดตำแหน่งของแพ็คเกจที่เกี่ยวข้อง
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # เปลี่ยนชื่อแพ็คเกจด้านล่างให้ตรงกับแพ็คเกจของคุณที่เก็บ map และ parameter
    my_nav_pkg_dir = get_package_share_directory('omni_carver_nav2')
    
    # กำหนดตำแหน่งไฟล์ configuration ต่างๆ
    rviz_config_file = os.path.join(my_nav_pkg_dir, 'rviz', 'navigation_defaut_view.rviz')
    map_yaml_file = os.path.join(my_nav_pkg_dir, 'maps', 'map_5.yaml')
    params_file = os.path.join(my_nav_pkg_dir, 'config', 'nav2_params.yaml')
    
    # สร้าง LaunchConfigurations สำหรับการกำหนด argument
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ประกาศ Launch Arguments
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Set to "True" to enable SLAM; "False" to use a pre-built map'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    # รวม launch file ของ nav2_bringup ซึ่งจะเริ่ม node หลักของ navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )
    
    # รวม RViz launch สำหรับการดูสถานะของ navigation (เลือกใช้งานได้)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file
        }.items()
    )
    
    # UNCOMMENT HERE FOR KEEPOUT DEMO
    # start_lifecycle_manager_cmd = Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_costmap_filters',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[{'use_sim_time': use_sim_time},
    #                     {'autostart': True},
    #                     {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}])

    # start_map_server_cmd = Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='filter_mask_server',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[params_file])

    # start_costmap_filter_info_server_cmd = Node(
    #         package='nav2_map_server',
    #         executable='costmap_filter_info_server',
    #         name='costmap_filter_info_server',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[params_file])
    
    # สร้าง launch description และเพิ่ม action ทั้งหมดลงไป
    ld = LaunchDescription()
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    # UNCOMMENT HERE FOR KEEPOUT DEMO
    # ld.add_action(start_lifecycle_manager_cmd)
    # ld.add_action(start_map_server_cmd)
    # ld.add_action(start_costmap_filter_info_server_cmd)
    
    return ld