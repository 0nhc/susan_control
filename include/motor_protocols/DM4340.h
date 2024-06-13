#ifndef DM4340_H
#define DM4340_H

// Motor protocols for DM-J4340-2EC motor. Fabricated by 达妙科技.

#include <can_msgs/Frame.h>
#include <utils/utils.h>
#include <vector>

#define POS_MIN_4340 -12.5f
#define POS_MAX_4340 12.5f
#define VEL_MIN_4340 -30.0f
#define VEL_MAX_4340 30.0f
#define KP_MIN_4340 0.0f
#define KP_MAX_4340 500.0f
#define KD_MIN_4340 0.0f
#define KD_MAX_4340 5.0f
#define TOR_MIN_4340 -10.0f
#define TOR_MAX_4340 10.0f

class DM4340
{
    public:
        DM4340();
        can_msgs::Frame encodeMITCommand(uint8_t motor_id, float position, float velocity, float kp, float kd, float torque);
        can_msgs::Frame encodeCalibrateCommand(uint8_t motor_id);
        can_msgs::Frame encodeDisableCommand(uint8_t motor_id);
        can_msgs::Frame encodeEnableCommand(uint8_t motor_id);
        float decodePositionFrame(can_msgs::Frame frame);

    private:
        can_msgs::Frame _dm4340_mit_command;
        can_msgs::Frame _dm4340_calibrate_command;
        can_msgs::Frame _dm4340_disable_frame;
        can_msgs::Frame _dm4340_enable_frame;
        float _position_state;
};

#endif