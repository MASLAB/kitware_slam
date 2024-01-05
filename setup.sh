#!/bin/bash

KITWARE_SLAM_DIR=$(pwd)

cd ..

# Install slam_toolbox
echo "Install slam_toolbox"
sudo apt -qq -y install ros-$ROS_DISTRO-slam-toolbox

# Install laser scan matcher
echo "Install laser scan matcher"
git clone https://github.com/AlexKaravaev/csm.git > /dev/null 2>&1
git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git > /dev/null 2>&1

# Setup LIDAR
_LDLIDAR_="LDLIDAR (small black one)"
_YDLIDAR_="YDLIDAR (big blue one)"

echo "Select which LIDAR to use:"

select lidar_opt in "$_LDLIDAR_" "$_YDLIDAR_"
do
    case $lidar_opt in
        $_LDLIDAR_)
            echo "Installing LDLIDAR"
            git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git > /dev/null 2>&1
            cd ldlidar_stl_ros2/scripts
            chmod +x ./create_udev_rules.sh
            ./create_udev_rules.sh > /dev/null 2>&1
            break
            ;;
        $_YDLIDAR_)
            echo "Installing YDLIDAR"
            git clone https://github.com/YDLIDAR/YDLidar-SDK.git > /dev/null 2>&1
            cd YDLidar-SDK
            mkdir build
            cd build
            cmake .. > /dev/null 2>&1
            make > /dev/null 2>&1
            sudo make install > /dev/null 2>&1
            cd ../..
            rm -rf YDLidar-SDK
            git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git > /dev/null 2>&1
            cd ydlidar_ros2_driver
            git checkout humble 2>&1
            cd startup 
            chmod +x ./initenv.sh
            sudo sh ./initenv.sh > /dev/null 2>&1
            break
            ;;
        *)
            echo "Invalid LIDAR, please select another one"
            ;;
    esac
done