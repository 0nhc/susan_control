#ifndef MG6012I36_H
#define MG6012I36_H

// Motor protocols for MG6012i36 motor. Fabricated by 瓴控科技.

#include <can_msgs/Frame.h>
#include <utils/utils.h>
#include <vector>

#define MAX_VEL 30.0f // degrees per second.

class MG6012I36
{
    public:
        MG6012I36();
        can_msgs::Frame encodePositionCommand(uint8_t motor_id, float position);
        can_msgs::Frame encodeCalibrateCommand(uint8_t motor_id);
        can_msgs::Frame encodeDisableCommand(uint8_t motor_id);
        can_msgs::Frame encodeEnableCommand(uint8_t motor_id);
        float decodePositionFrame(can_msgs::Frame frame);

    private:
        can_msgs::Frame _mg6012_position_command;
        can_msgs::Frame _mg6012_calibrate_command;
        can_msgs::Frame _mg6012_disable_frame;
        can_msgs::Frame _mg6012_enable_frame;
        float _position_state;
        uint16_t _uint16_max_vel;
};

#endif