#include <motor_protocols/MG4010I36.h>
#include <iostream>

MG4010I36::MG4010I36()
{
    _uint16_max_vel = MAX_VEL * 36.0; //减速比36
};

can_msgs::Frame MG4010I36::encodePositionCommand(uint8_t motor_id, float position)
{
    // 默认函数传入弧度制，先转化成角度值。
    position = position / M_PI * 180.0;
    int32_t int32_position = (int32_t)(position * 100.0 * 36.0); // 360度对应36000LSB，结合减速比36
    _mg4010_position_command.id = 0x140 + motor_id;
    _mg4010_position_command.dlc = 8;
    _mg4010_position_command.data[0] = 0xA4;
    _mg4010_position_command.data[1] = 0x00;
    _mg4010_position_command.data[2] = _uint16_max_vel;
    _mg4010_position_command.data[3] = _uint16_max_vel >> 8;
    _mg4010_position_command.data[4] = int32_position;
    _mg4010_position_command.data[5] = int32_position >> 8;
    _mg4010_position_command.data[6] = int32_position >> 16;
    _mg4010_position_command.data[7] = int32_position >> 24;

    return _mg4010_position_command;
};

can_msgs::Frame MG4010I36::encodeCalibrateCommand(uint8_t motor_id)
{
    _mg4010_calibrate_command.id = 0x140 + motor_id;
    _mg4010_calibrate_command.dlc = 8;
    _mg4010_calibrate_command.data[0] = 0x19;
    _mg4010_calibrate_command.data[1] = 0x00;
    _mg4010_calibrate_command.data[2] = 0x00;
    _mg4010_calibrate_command.data[3] = 0x00;
    _mg4010_calibrate_command.data[4] = 0x00;
    _mg4010_calibrate_command.data[5] = 0x00;
    _mg4010_calibrate_command.data[6] = 0x00;
    _mg4010_calibrate_command.data[7] = 0x00;

    return _mg4010_calibrate_command;
};

can_msgs::Frame MG4010I36::encodeDisableCommand(uint8_t motor_id)
{
    _mg4010_disable_frame.id = 0x140 + motor_id;
    _mg4010_disable_frame.dlc = 8;
    _mg4010_disable_frame.data[0] = 0x80;
    _mg4010_disable_frame.data[1] = 0x00;
    _mg4010_disable_frame.data[2] = 0x00;
    _mg4010_disable_frame.data[3] = 0x00;
    _mg4010_disable_frame.data[4] = 0x00;
    _mg4010_disable_frame.data[5] = 0x00;
    _mg4010_disable_frame.data[6] = 0x00;
    _mg4010_disable_frame.data[7] = 0x00;

    return _mg4010_disable_frame;
};

can_msgs::Frame MG4010I36::encodeEnableCommand(uint8_t motor_id)
{
    _mg4010_enable_frame.id = 0x140 + motor_id;
    _mg4010_enable_frame.dlc = 8;
    _mg4010_enable_frame.data[0] = 0x88;
    _mg4010_enable_frame.data[1] = 0x00;
    _mg4010_enable_frame.data[2] = 0x00;
    _mg4010_enable_frame.data[3] = 0x00;
    _mg4010_enable_frame.data[4] = 0x00;
    _mg4010_enable_frame.data[5] = 0x00;
    _mg4010_enable_frame.data[6] = 0x00;
    _mg4010_enable_frame.data[7] = 0x00;

    return _mg4010_enable_frame;
};

float MG4010I36::decodePositionFrame(can_msgs::Frame frame)
{
    Convert16 cvt16;
    cvt16.data[0] = frame.data[6];
    cvt16.data[1] = frame.data[7];

    _position_state = (float)cvt16.to_int16 / 16384.0 / 2.0 * M_PI; // 14bit编码器分辨率16384，我也不知道为啥除以2.0之后才是准确的位置

    return _position_state;
};