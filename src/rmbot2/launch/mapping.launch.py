import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command



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

    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fastlio',
        description='Choose lio alogrithm: fastlio or pointlio')

    
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')


    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav_rviz = LaunchConfiguration('nav_rviz')
    slam_toolbox_mapping_file_dir = os.path.join(rmbot_dir, 'config', 'simulation', 'mapper_params_online_async_sim.yaml')


    mac_rm_simulation_launch_dir = os.path.join(get_package_share_directory('rmbot2'))
    

    start_rm_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mac_rm_simulation_launch_dir, 'launch/launch_world.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            # 'robot_description': robot_description,
            'rviz': 'false'}.items()
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
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

    segmentation_params = os.path.join(rmbot_dir, 'config', 'simulation', 'segmentation_sim.yaml')
    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params]
    )


    # pointcloud to laser scan
    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        # remappings=[('cloud_in',  ['segmentation/obstacle']),
        remappings=[('cloud_in',  ['/livox/lidar_PointCloud2']),          
                    ('scan',  ['/scan'])],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.001,
            'min_height': -1.0,
            'max_height': 2.0,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.01,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )


    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
    )


    rm_nav_bringup_dir = get_package_share_directory('rm_nav_bringup')
    nav2_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world]), ".yaml"
    nav2_params_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'simulation', 'nav2_params_sim.yaml')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz}.items()
    )

    bringup_fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'spin_speed': 5.0 # rad/s
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_LIO_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_mode_cmd)

    ld.add_action(start_rm_simulation)
    # ld.add_action(bringup_imu_complementary_filter_node)
    # ld.add_action(bringup_LIO_group)

    # skip the ground segmentation for now       
    # ld.add_action(bringup_linefit_ground_segmentation_node)

    # ld.add_action(bringup_pointcloud_to_laserscan_node) 

    ld.add_action(bringup_fake_vel_transform_node)

    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)
    return ld
