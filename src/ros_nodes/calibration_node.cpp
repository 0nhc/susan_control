#include "ros_interface/calibration.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh("");
    CALIBRATION calibration(&nh);
    calibration.calibrate();
    ros::spin();
    return 0;
}