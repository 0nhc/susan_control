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
```sh
# Remember to set up your CANable devices at first.
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up

# The move the robotic arm to the zero position, and launch the calibration node:
cd <your_ws>
source devel/setup.bash
roslaunch susan_control calibration.launch
```

## 4. Launch the Hardware Interface
```sh
# Remember to set up your CANable devices at first.
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up

# Then
cd <your_ws>
source devel/setup.bash
roslaunch susan_control susan_control.launch hardware_type:=real
```
Note that you can specify the hardware type between "fake" and "real".
* fake: It can only be controlled in position mode. Joint states are the same as the joint commands you give.
* real: It can only be controlled in position mode. Joint states are read from CAN Bus.

## 5. Torque Control Mode (开发中，未完成)
目前只在[torque_hardware_interface.cpp](./src/ros_interface/torque_hardware_interface.cpp)里接入了机械臂第一个电机的力控，但其实所有7个电机的力控接口都已经写好，因为没有做好动力学控制所以没有写完完整的力控接口。可以根据目前的[torque_hardware_interface.cpp](./src/ros_interface/torque_hardware_interface.cpp)代码以及参考[susan_hardware_interface.cpp](./src/ros_interface/susan_hardware_interface.cpp)将其扩展成7个电机的力控接口。
```sh
# Remember to set up your CANable devices at first.
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up

# Then
cd <your_ws>
source devel/setup.bash
roslaunch susan_control torque_control.launch hardware_type:=real
```