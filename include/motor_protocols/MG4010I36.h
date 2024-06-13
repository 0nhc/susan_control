#ifndef MG4010I36_H
#define MG4010I36_H

// Motor protocols for MG4010i36 motor. Fabricated by 瓴控科技.

#include <can_msgs/Frame.h>
#include <utils/utils.h>
#include <vector>

#define MAX_VEL 60.0f // degrees per second.

class MG4010I36
{
    public:
        MG4010I36();
        can_msgs::Frame encodePositionCommand(uint8_t motor_id, float position);
        can_msgs::Frame encodeCalibrateCommand(uint8_t motor_id);
        can_msgs::Frame encodeDisableCommand(uint8_t motor_id);
        can_msgs::Frame encodeEnableCommand(uint8_t motor_id);
        float decodePositionFrame(can_msgs::Frame frame);

    private:
        can_msgs::Frame _mg4010_position_command;
        can_msgs::Frame _mg4010_calibrate_command;
        can_msgs::Frame _mg4010_disable_frame;
        can_msgs::Frame _mg4010_enable_frame;
        float _position_state;
        uint16_t _uint16_max_vel;
};

#endif