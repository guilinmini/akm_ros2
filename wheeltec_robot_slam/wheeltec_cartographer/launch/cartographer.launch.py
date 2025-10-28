import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
        default=os.path.join(get_package_share_directory('wheeltec_cartographer') , 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')


    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # lidar_launch_dir = get_package_share_directory('oradar_lidar')
    # lidar_launch_dir = os.path.join(lidar_launch_dir, 'launch')

    ros2_laser_scan_matcher_launch_dir = get_package_share_directory('ros2_laser_scan_matcher')
    ros2_laser_scan_matcher_launch_dir = os.path.join(ros2_laser_scan_matcher_launch_dir,"launch")

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
            launch_arguments={'carto_slam': 'true'}.items(),
    )
    # wheeltec_lidar = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'ms200_scan.launch.py')),
    # )

    # Include the ros2_laser_scan_matcher launch file
    ros2_laser_scan_matcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros2_laser_scan_matcher_launch_dir,'laser_scan_matcher_launch.py')),
    )
    return LaunchDescription([
    wheeltec_robot,ros2_laser_scan_matcher,

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename],
            remappings=[
                  ('scan', 'scan'),  #
                  # ('imu','/imu/data'),
                  ('odom','odom_laser')
              ]
            ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',

            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),

    ])
