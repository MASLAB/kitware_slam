import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  kitware_slam_dir = get_package_share_directory('kitware_slam')
  
  # Lidar configuration
  lidar_config = os.path.join(
    kitware_slam_dir,
    'config',
    'ldlidar.yaml'
  )
  
  # LIDAR node
  lidar_node = Node(
    package='ldlidar_stl_ros2',
    executable='ldlidar_stl_ros2_node',
    name='lidar_node',
    output='screen',
    parameters=[lidar_config]
  )

  # base_laser to laser_frame tf node
  base_laser_to_laser_frame_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_laser_to_laser_frame_tf_node',
    arguments=['0','0','0.018','0','0','0','base_laser','laser_frame']
  )

  # Common launch
  common_launch = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource([
      kitware_slam_dir,
      '/launch/common.launch.py'
    ])
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()
  ld.add_action(lidar_node)
  ld.add_action(base_laser_to_laser_frame_tf_node)
  ld.add_action(common_launch)

  return ld