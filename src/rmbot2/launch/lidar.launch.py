import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition


def generate_launch_description():
    
    rmbot_dir= get_package_share_directory('rmbot2')
    fastlio_mid360_params = os.path.join(rmbot_dir, 'config', 'simulation', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(rmbot_dir, 'rviz', 'fastlio.rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='false',
        description='Visualize FAST_LIO or Point_LIO cloud_map if true')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='RMUL',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_io_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fastlio',
        description='Select LIO mapping method')


    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    mac_rm_simulation_launch_dir = os.path.join(get_package_share_directory('rmbot2'))    

    start_rm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mac_rm_simulation_launch_dir, 'launch/launch_world.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            # 'robot_description': robot_description,
            'rviz': 'true'}.items()
    )

    bringup_LIO_group = GroupAction([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                # Useless arguments, provided by LIO in publish_odometry() function
                # '--x', '0.0',
                # '--y', '0.0',
                # '--z', '0.0',
                # '--roll', '0.0',
                # '--pitch', '0.0',
                # '--yaw', '0.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'lidar_odom'
            ],
        ),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'fastlio'),
            actions=[
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[
                    fastlio_mid360_params,
                    {use_sim_time: use_sim_time}
                ],
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', fastlio_rviz_cfg_dir],
                condition = IfCondition(LaunchConfiguration('lio_rviz'))
            ),
        ])
    ])


    ld = LaunchDescription()
    ld.add_action(declare_io_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)

    ld.add_action(start_rm_simulation)

    ld.add_action(bringup_LIO_group)
    
    return ld
