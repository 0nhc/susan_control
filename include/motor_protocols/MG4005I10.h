#ifndef MG4005I10_H
#define MG4005I10_H

// Motor protocols for MG4005i10 motor. Fabricated by 瓴控科技.

#include <can_msgs/Frame.h>
#include <utils/utils.h>
#include <vector>

class MG4005I10
{
    public:
        MG4005I10();
        can_msgs::Frame encodeTorqueCommand(uint8_t motor_id, float torque);
        float decodeVelocityFrame(can_msgs::Frame frame);

    private:
        can_msgs::Frame _torque_command;
        float _velocity_state;
};

#endif