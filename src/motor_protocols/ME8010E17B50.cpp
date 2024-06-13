#include <motor_protocols/ME8010E17B50.h>
#include <iostream>

ME8010E17B50::ME8010E17B50()
{
    for(int i=0; i<NUM_CALIBRATE_FRAMES; i++)
    {
        can_msgs::Frame _calibrate_frame;
        _calibrate_command.push_back(_calibrate_frame);
    }
};

can_msgs::Frame ME8010E17B50::encodePositionCommand(uint8_t motor_id, float position)
{
    // 默认函数传入弧度制，先转化成角度值。
    position = position / M_PI * 180.0;
    int32_t int32_position = (int32_t)(-position * 4642.133333333); // 15-bit编码器一圈对应32768个脉冲，乘以51减速比后电机输出端一圈对应1671168个脉冲，因此1度对应1671168/360=4642.133333333个脉冲
    _position_command.id = 0x200 + motor_id;
    _position_command.dlc = 7;
    _position_command.data[0] = 0x2F;
    _position_command.data[1] = 0x00;
    _position_command.data[2] = 0x01;
    _position_command.data[3] = int32_position;
    _position_command.data[4] = int32_position >> 8;
    _position_command.data[5] = int32_position >> 16;
    _position_command.data[6] = int32_position >> 24;

    return _position_command;
};

std::vector<can_msgs::Frame> ME8010E17B50::encodeCalibrateCommand(uint8_t motor_id)
{
    can_msgs::Frame _calibrate_frame;
    _calibrate_frame.id = 0x600 + motor_id;
    _calibrate_frame.dlc = 6;

    _calibrate_frame.data[0] = 0x2B;
    _calibrate_frame.data[1] = 0x0A;
    _calibrate_frame.data[2] = 0x26;
    _calibrate_frame.data[3] = 0x00;
    _calibrate_frame.data[4] = 0x66;
    _calibrate_frame.data[5] = 0xEA;
    _calibrate_command[0] = _calibrate_frame;

    _calibrate_frame.data[0] = 0x2B;
    _calibrate_frame.data[1] = 0x0A;
    _calibrate_frame.data[2] = 0x26;
    _calibrate_frame.data[3] = 0x00;
    _calibrate_frame.data[4] = 0x70;
    _calibrate_frame.data[5] = 0xEA;
    _calibrate_command[1] = _calibrate_frame;

    return _calibrate_command;
};

can_msgs::Frame ME8010E17B50::encodeDisableCommand(uint8_t motor_id)
{
    _disable_command.id = 0x00;
    _disable_command.dlc = 2;
    _disable_command.data[0] = 0x02;
    _disable_command.data[1] = motor_id;

    return _disable_command;
};

can_msgs::Frame ME8010E17B50::encodeEnableCommand(uint8_t motor_id)
{
    _enable_command.id = 0x00;
    _enable_command.dlc = 2;
    _enable_command.data[0] = 0x01;
    _enable_command.data[1] = motor_id;

    return _enable_command;
};

float ME8010E17B50::decodePositionFrame(can_msgs::Frame frame)
{
    Convert32 cvt32;
    cvt32.data[0] = frame.data[0];
    cvt32.data[1] = frame.data[1];
    cvt32.data[2] = frame.data[2];
    cvt32.data[3] = frame.data[3];

    _position_state = -(cvt32.to_int32 / 4642.133333333 / 180.0 * M_PI);

    return _position_state;
};