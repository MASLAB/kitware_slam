import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  kitware_slam_dir = get_package_share_directory('kitware_slam')
  
  # Slam configuration
  slam_config = os.path.join(
    kitware_slam_dir,
    'config',
    'mapper_params_online_sync.yaml'
  )

  base_link_to_base_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_tf_node',
    arguments=['--x', '0.06',
               '--y', '0',
               '--z', '0.1',
               '--yaw', '0', 
               '--pitch', '0',
               '--roll', '0',
               '--frame-id', 'base_link',
               '--child-frame-id', 'base_laser'
              ], # Edit for correct location of laser base with respect to center of robot
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

  sync_slam_toolbox_node = Node(
    parameters=[
      slam_config,
      {'use_sim_time' : False}
    ],
    package='slam_toolbox',
    executable='sync_slam_toolbox_node',
    name='slam_toolbox',
    output='screen'
  )

  # Kitbot node
  kitbot_node = Node(
    package='kitware',
    executable='kitbot',
    output='screen'
  )

  # Differential driver node
  differential_driver_node = Node(
    package='kitware_slam',
    executable='differential_driver_node',
    output='screen'
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()
  ld.add_action(base_link_to_base_laser_tf_node)
  ld.add_action(laser_scan_matcher_node)
  ld.add_action(sync_slam_toolbox_node)
  ld.add_action(kitbot_node)
  ld.add_action(differential_driver_node)

  return ld
