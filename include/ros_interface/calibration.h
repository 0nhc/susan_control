#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "ros/ros.h"
#include <can_msgs/Frame.h>
#include <motor_protocols/ME8010E17B50.h>
#include <motor_protocols/MG4010I36.h>
#include <memory>

#define NUM_JOINTS 6
#define PUB_TIMES 1

class CALIBRATION
{
    public:
        CALIBRATION(ros::NodeHandle *nh);
        void calibrate();

    private:
        ros::Publisher can_frame_publisher_;

        MG4010I36 mg4010_protocols_;
        can_msgs::Frame mg4010_calibration_frame_;

        ME8010E17B50 me8010_protocols_;
        std::vector<can_msgs::Frame> me8010_calibration_frames_;
};

#endif