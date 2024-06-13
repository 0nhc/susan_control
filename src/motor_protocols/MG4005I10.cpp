#include <motor_protocols/MG4005I10.h>
#include <iostream>

MG4005I10::MG4005I10()
{
};

can_msgs::Frame MG4005I10::encodeTorqueCommand(uint8_t motor_id, float torque)
{
    float current = torque * 100.0 / 6.0 / 10.0; // 瓴控给的MG4005i0电机扭矩常数为0.06N.M/A，外加考虑10：1的减速比
    int16_t current_buffer = (int16_t)(current * 2000.0 / 32.0); // 瓴控的CAN通信协议里给的转矩闭环控制映射，-32A~32A对应数值范围-2000~2000
    _torque_command.id = 0x140 + motor_id;
    _torque_command.dlc = 8;
    _torque_command.data[0] = 0xA1;
    _torque_command.data[1] = 0x00;
    _torque_command.data[2] = 0x00;
    _torque_command.data[3] = 0x00;
    _torque_command.data[4] = current_buffer;
    _torque_command.data[5] = current_buffer >> 8;
    _torque_command.data[6] = 0x00;
    _torque_command.data[7] = 0x00;

    return _torque_command;
};

float MG4005I10::decodeVelocityFrame(can_msgs::Frame frame)
{
    Convert16 cvt16;
    cvt16.data[0] = frame.data[4];
    cvt16.data[1] = frame.data[5];

    _velocity_state = (float)cvt16.to_int16 / 10.0 / 180.0 * M_PI / 2.0; // 瓴控的CAN通信协议里给的速度反馈定义：1dps/LSB，外加考虑10：1的减速比。但是我也不知道为什么只有在除以2.0之后才是比较准确的速度（弧度制）。

    return _velocity_state;
};