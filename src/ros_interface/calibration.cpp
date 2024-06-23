#include <ros_interface/calibration.h>

CALIBRATION::CALIBRATION(ros::NodeHandle *nh)
{
    can_frame_publisher_ = nh->advertise<can_msgs::Frame>("sent_messages", 1);
    usleep(1000000); // wait 1 s for initializing
    // Enable DM Motors
    for(int i=2; i<7; i++)
    {
        dm4340_enable_frame_ = dm4340_protocols_.encodeEnableCommand(i+1);
        dm4340_enable_frame_.header.stamp = ros::Time::now();
        dm4340_enable_frame_.header.frame_id = "DM4340";
        can_frame_publisher_.publish(dm4340_enable_frame_);
        usleep(1000);
    }
};

void CALIBRATION::calibrate()
{
    for(int i=0; i<NUM_JOINTS; i++)
    {
        if(i>=0 && i<2)
        {
            mg6012_calibration_frame_ = mg6012_protocols_.encodeCalibrateCommand(i+1);
            for(int j=0; j<PUB_TIMES; j++)
            {
                mg6012_calibration_frame_.header.stamp = ros::Time::now();
                mg6012_calibration_frame_.header.frame_id = "MG6012I36";
                can_frame_publisher_.publish(mg6012_calibration_frame_);
                usleep(100000);
            }
        }
        else if(i>=2 && i<7)
        {
            dm4340_calibration_frame_ = dm4340_protocols_.encodeCalibrateCommand(i+1);
            for(int j=0; j<PUB_TIMES; j++)
            {
                dm4340_calibration_frame_.header.stamp = ros::Time::now();
                dm4340_calibration_frame_.header.frame_id = "DM4340";
                can_frame_publisher_.publish(dm4340_calibration_frame_);
                usleep(100000);
            }
        }
        ROS_INFO("Successfully calibrated motor %i.", i+1);
    }
    ROS_INFO("Successfully calibrated all motors!");
}
