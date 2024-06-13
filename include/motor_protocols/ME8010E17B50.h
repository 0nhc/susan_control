#ifndef ME8010E17B50_H
#define ME8010E17B50_H

// Motor protocols for ME8010E17B50 motor. Fabricated by 时代超群.

#include <can_msgs/Frame.h>
#include <utils/utils.h>
#include <vector>

#define NUM_CALIBRATE_FRAMES 2

class ME8010E17B50
{
    public:
        ME8010E17B50();
        can_msgs::Frame encodePositionCommand(uint8_t motor_id, float position);
        std::vector<can_msgs::Frame> encodeCalibrateCommand(uint8_t motor_id);
        can_msgs::Frame encodeDisableCommand(uint8_t motor_id);
        can_msgs::Frame encodeEnableCommand(uint8_t motor_id);
        float decodePositionFrame(can_msgs::Frame frame);

    private:
        std::vector<can_msgs::Frame> _calibrate_command;
        can_msgs::Frame _position_command;
        can_msgs::Frame _disable_command;
        can_msgs::Frame _enable_command;
        float _position_state;
};

#endif