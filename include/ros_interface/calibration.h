#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "ros/ros.h"
#include <can_msgs/Frame.h>
#include <motor_protocols/MG6012I36.h>
#include <motor_protocols/DM4340.h>
#include <memory>

#define NUM_JOINTS 7
#define PUB_TIMES 1

class CALIBRATION
{
    public:
        CALIBRATION(ros::NodeHandle *nh);
        void calibrate();

    private:
        ros::Publisher can_frame_publisher_;

        MG6012I36 mg6012_protocols_;
        can_msgs::Frame mg6012_calibration_frame_;

        DM4340 dm4340_protocols_;
        can_msgs::Frame dm4340_calibration_frame_;
        can_msgs::Frame dm4340_enable_frame_;
};

#endif