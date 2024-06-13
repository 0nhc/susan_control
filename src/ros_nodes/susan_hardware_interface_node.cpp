#include "ros_interface/susan_hardware_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "susan_hardware_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    SusanHardwareInterface Susan(nh);
    spinner.spin();
    return 0;
}