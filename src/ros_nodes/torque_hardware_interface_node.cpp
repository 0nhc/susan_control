#include "ros_interface/torque_hardware_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "torque_hardware_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    TorqueHardwareInterface TorqueControl(nh);
    spinner.spin();
    return 0;
}