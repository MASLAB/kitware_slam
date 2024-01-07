# Kitware SLAM
SLAM and navigation implementation for kitbot
* Install necessary packages for SLAM with LIDAR:
  * [ldlidar](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
  * [ydlidar](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble)
  * [scan matcher](https://github.com/AlexKaravaev/csm)
  * [ros2 laser scan matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher)
  * [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
* Use [kitware](https://github.com/MASLAB/kitware) to drive the robot to a pose set by `rviz2`

## Setup
1. Clone into the `src` directory of your colcon workspace for ROS
2. Make sure you have cloned [kitware](https://github.com/MASLAB/kitware) and [kitware_interface](https://github.com/MASLAB/kitware_interface) in `src` as well
3. `cd` in to the `kitware_slam` and run `setup.sh`
4. Run `colcon build` from the workspace folder

The following steps should already be done on the hardware we give students.
1. Make sure you have [TAMProxy-Firmware](https://github.com/MASLAB/TAMProxy-Firmware) running on a teensy
2. Install [TAMProxy-pyHost](https://github.com/MASLAB/TAMProxy-pyHost)

## Modify for your robot
* Change the transformation for `base_link_to_base_laser_tf_node` in `launch/common.launch.py` to reflect the location of the LIDAR's base to the center of your robot. The argument format is
  ```python
  [ x, y, z, roll, pitch, yaw, parent_frame, child_frame]
  ```
  or
  ```python
  [ x, y, z, qx, qy, qz, qw, parent_frame, child_frame]
  ```
* Tuning the PID and error tolerance constants in `kitware/differential_driver.py` for desired performance/accuracy
> [!IMPORTANT]
> Run `colcon build` from the workspace folder to update the changes after modifying. Else the changes will not be applied

## Running
1. Make sure the robot is running off the battery and not attached to anything
2. `ssh -X` into the robot
3. `cd` into your ROS workspace
4. Run `source install/setup.bash` to setup your environment to use the built packages  
5. Launch  
  * If using LDLIDAR: run `ros2 launch kitware_slam ldlidar_slam.launch.py`
  * If using YDLIDAR: run `ros2 launch kitware_slam ldlidar_slam.launch.py`
6. Open another terminal and `ssh -X` into the robot again
7. Run `rviz2`
8. Open `rviz` configuration :TODO:
9. Set a goal pose
10. Cross your finger that the robot will go

## Troubleshoot
* Make sure the `kitbot` works with keyboard. If not then troubleshoot that first (pin settings in `kitbot.py` is correct, motor wiring is correct)
