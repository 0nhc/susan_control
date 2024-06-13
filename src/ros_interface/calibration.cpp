#include <ros_interface/calibration.h>

CALIBRATION::CALIBRATION(ros::NodeHandle *nh)
{
    can_frame_publisher_ = nh->advertise<can_msgs::Frame>("sent_messages", 1);
    usleep(1000000); // wait 1 s for initializing
};

void CALIBRATION::calibrate()
{
    for(int i=0; i<NUM_JOINTS; i++)
    {
        if(i>=0 && i<3)
        {
            me8010_calibration_frames_ = me8010_protocols_.encodeCalibrateCommand(i+1);
            for(int j=0; j<PUB_TIMES; j++)
            {
                me8010_calibration_frames_[0].header.stamp = ros::Time::now();
                me8010_calibration_frames_[0].header.frame_id = "ME8010E17B50";
                can_frame_publisher_.publish(me8010_calibration_frames_[0]);
                usleep(100000);
                
                me8010_calibration_frames_[1].header.stamp = ros::Time::now();
                me8010_calibration_frames_[1].header.frame_id = "ME8010E17B50";
                can_frame_publisher_.publish(me8010_calibration_frames_[1]);
                usleep(100000);
            }
        }
        else if(i>=3 && i<6)
        {
            mg4010_calibration_frame_ = mg4010_protocols_.encodeCalibrateCommand(i+1);
            for(int j=0; j<PUB_TIMES; j++)
            {
                mg4010_calibration_frame_.header.stamp = ros::Time::now();
                mg4010_calibration_frame_.header.frame_id = "MG4010I36";
                can_frame_publisher_.publish(mg4010_calibration_frame_);
                usleep(100000);
            }
        }
        ROS_INFO("Successfully calibrated motor %i.", i+1);
    }
    ROS_INFO("Successfully calibrated all motors!");
}
