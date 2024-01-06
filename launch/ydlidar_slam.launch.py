import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
  kitware_slam_dir = get_package_share_directory('kitware_slam')
  
  # Lidar configuration
  lidar_config = os.path.join(
    kitware_slam_dir,
    'config',
    'ydlidar.yaml'
  )
  
  # LIDAR node
  lidar_node = LifecycleNode(
    package='ydlidar_ros2_driver',
    executable='ydlidar_ros2_driver_node',
    name='lidar_node',
    output='screen',
    emulate_tty=True,
    parameters=[lidar_config],
    namespace='/',
  )

  # base_laser to laser_frame tf node
  base_laser_to_laser_frame_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_laser_to_laser_frame_tf_node',
    arguments=['0','0','0.02','0','0','0','1','base_laser','laser_frame']
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