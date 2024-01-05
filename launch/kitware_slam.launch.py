import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # RViZ2 settings
  rviz2_config = os.path.join(
    get_package_share_directory('ldlidar_stl_ros2'),
    'rviz2',
    'ldlidar.rviz'
  )

  rviz2_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2_show_ld19',
    arguments=['-d',rviz2_config],
    output='screen'
  )

  ldlidar_launch = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource([
      get_package_share_directory('ldlidar_stl_ros2'),
      '/launch/ld19.launch.py'
    ])
  )

  laser_scan_matcher_node = Node(
    package='ros2_laser_scan_matcher',
    executable='laser_scan_matcher',
    name='laser_scan_matcher_node',
    parameters=[{
      'publish_odom' : 'odom',
      'publish_tf'   : True,
      'laser_frame'  : 'base_laser'
    }],
    output='screen'
  )

  slam_config = os.path.join(
    get_package_share_directory('kitware_slam'),
    'config',
    'mapper_params_online_sync.yaml'
  )

  async_slam_toolbox_node = Node(
    parameters=[
      slam_config,
      {'use_sim_time' : False}
    ],
    package='slam_toolbox',
    executable='sync_slam_toolbox_node',
    name='slam_toolbox',
    output='screen'
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(rviz2_node)
  ld.add_action(ldlidar_launch)
  ld.add_action(laser_scan_matcher_node)
  ld.add_action(async_slam_toolbox_node)

  return ld