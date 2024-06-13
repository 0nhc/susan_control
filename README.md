# susan_control
Hardware Interface in ROS.

## 1. Prerequisites
* Ubuntu 20.04.
* ROS Noetic.
* [CANable](https://canable.io/) Devices.
* Install [susan_description](https://github.com/0nhc/susan_description).

## 2. Installation
```sh
cd <your_ws>/src
git clone https://github.com/0nhc/susan_control.git
cd ..
rosdep install --from-path src --ignore-src -r -y
catkin_make
```

## 3. Calibration.
* Remember to set up your CANable devices.
```sh
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up
```
