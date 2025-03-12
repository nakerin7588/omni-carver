# omni-carver

https://github.com/user-attachments/assets/3435d6dd-67d6-426c-801f-3730e0f8ce20


<!-- TABLE OF CONTENTS -->
# Table of Contents
<ol>
    <li>
        <a href="#about-the-project">About The Project</a>
        <ul>
            <li><a href="#system-architecture">System Architecture</a></li>
        </ul>
    </li>
    <li>
        <a href="#robot-setup">Robot setup</a>
        <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
            <ul>
                <li><a href="#python-packages">Python packages</a></li>
                <li><a href="#ros2-packages">ROS2 packages</a></li>
            </ul>
        <li><a href="#installation">Installation</a></li>
        </ul>
    </li>
    <li><a href="#contributors">Contributors</a></li>
</ol>


# About The Project

This project is the part of open-topic research of FRA361 class that focus about how to getting map with actual robot by create new firmware of robot to using slam toolbox for mapping.

# Hardware

## 1. 3 omni-wheels mobile robot.

## 2. Laser Sensor

- [LiDAR LD06](https://th.rs-online.com/web/p/sensor-development-tools/2037609)

## 3. IMU

- [BNO055 USB stick](https://www.digikey.co.th/th/products/detail/bosch-sensortec/BNO055-USB-STICK/6136288?srsltid=AfmBOopQXxThDLAN2mAPddHxnJptcNuUyhAxiubvHIMDygA-2x8FSyLa)

## 4. Microcontroller

- [ESP32 Wroom 32](https://www.arduitronics.com/product/5007/%E0%B8%9A%E0%B8%AD%E0%B8%A3%E0%B9%8C%E0%B8%94-esp-32-nodemcu-esp-wroom-32-wi-fi-and-bluetooth-dual-core-ch9102x-30-pin)

## 5. Processor

- Intel NUC

# Firmware setup(ESP32 Firmware)

ESP32 is the microcontroller for our robot's low level control that have a PID controller for each motor to control the motor to reach velocity setpoint.

> !NOTE
> You can find the ESP32 firmware via this <a href="src/esp32_firmware/esp32/">link</a>. Additionally, I connect ESP32 and NUC by using <a href="https://github.com/pyserial/pyserial">pyserial</a>.

# Software setup(ROS2)

For this robot I use <a href="https://docs.ros.org/en/jazzy/index.html">ROS2 Jazzy</a> to be our middleware to build this robot. And to use this robot to do this task I have to create some new nodes for make this robot can teleop with [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard), then do mapping with slam toolbox.

> !WARNING
> For everything that explain below, you need to install dependencies that tell in that section if you want to use this robot.

## 1. Arduino serial node

See in <a href="src/omni_carver_arduino_serial/scripts/arduino_serial_node_script.py">arduino_serial_node_script.py</a>. This node has task to stream joint states data and send joint velocity command from/to ESP32.

## 2. BNO055 USB stick node

See in <a href="src/bno055_usb_stick/scripts/bno055_usb_stick_node_script.py">bno055_usb_stick_node</a>. This node has task to stream imu's data from BNO055 USB stick via [bno055_usb_stick_py](https://github.com/selyunin/bno055_usb_stick_py).

## 3. ldlidar_stl_ros2

This package is from LDROBOT who is develop a LD06 lidar. To use LD06, I use this package to communicate between ROS2 and LD06. You can find original github [here](https://github.com/rudislabs/ldlidar_stl_ros2/tree/pr-binning).

> !COUTION
> To use with slam toolbox, make sure you clone this package from `pr-binning` branch(Same as upper link).

## 4. omni_drive_node

See in <a href="src/omni_carver_controller/scripts/omni_drive_node_script.py">omni_drive_node_script.py</a>. This node is implement kinematics model of 3 omni-wheels mobile robot to calculate twist at base frame of robot to wheel velocity at wheel frame call inverse kinematics and calculate wheel odometry of robot via forward kinematics.

### Inverse kinematics

### Forward kinematics

## 5. omni_carver_description

See in <a href="src/omni_carver_description/launch/description.launch.py">description.launch.py</a>. This package is for create transformation of robot via URDF file to visualize by rviz2 and calculate some data.

## 6. omni_carver_localization

See in <a href="src/omni_carver_localization/launch/ekf.launch.py">ekf.launch.py</a>. This package has contain launch file for EKF from robot localization package to filt wheel odometry and imu togeter to make odometry of robot more smooth and accurate. For robot localization config, I use this guide to custom my config.

> !WARNING
> At this moment robot have a lot of error form sensor such as IMU or wheel encoder. So it need to tune more to make odometry more accurate.

## 7. omni_carver_slam

See in <a href="src/omni_carver_slam/launch/mapping.launch.py">mapping.launch.py</a>. This pacakge has contain launch file for mapping and save map via slam toolbox and nav2.

# Mapping



# Demonstrate



# Issues

1. 
2. 
3. 
4. 
5. 



# Contributors
- Nakarin Jettanatummajit (65340500033)