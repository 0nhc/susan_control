#include <motor_protocols/DM4340.h>
#include <iostream>

DM4340::DM4340()
{
};

can_msgs::Frame DM4340::encodeMITCommand(uint8_t motor_id, float position, float velocity, float kp, float kd, float torque)
{
    float _position = _limit(position, POS_MIN_4340, POS_MAX_4340);
    float _velocity = _limit(velocity, VEL_MIN_4340, VEL_MAX_4340);
    float _kp = _limit(kp, KP_MIN_4340, KP_MAX_4340);
    float _kd = _limit(kd, KD_MIN_4340, KD_MAX_4340);
    float _torque = _limit(torque, TOR_MIN_4340, TOR_MAX_4340);

    uint16_t pos_tmp = _float_to_uint(_position, POS_MIN_4340, POS_MAX_4340, 16);
    uint16_t vel_tmp = _float_to_uint(_velocity, VEL_MIN_4340, VEL_MAX_4340, 12);
    uint16_t kp_tmp = _float_to_uint(_kp, KP_MIN_4340, KP_MAX_4340, 12);
    uint16_t kd_tmp = _float_to_uint(_kd, KD_MIN_4340, KD_MAX_4340, 12);
    uint16_t tor_tmp = _float_to_uint(_torque, TOR_MIN_4340, TOR_MAX_4340, 12);

    _dm4340_mit_command.id = motor_id;
    _dm4340_mit_command.dlc = 8;
    _dm4340_mit_command.data[0] = (unsigned char)(pos_tmp >> 8);
    _dm4340_mit_command.data[1] = (unsigned char)(pos_tmp & 0xFF);
    _dm4340_mit_command.data[2] = (unsigned char)((vel_tmp >> 4) & 0xFF);
    _dm4340_mit_command.data[3] = (unsigned char)((unsigned char)(((vel_tmp & 0xF) << 4) & 0xFF) | ((kp_tmp >> 8) & 0xFF));
    _dm4340_mit_command.data[4] = (unsigned char)(kp_tmp & 0xFF);
    _dm4340_mit_command.data[5] = (unsigned char)((kd_tmp >> 4) & 0xFF);
    _dm4340_mit_command.data[6] = (unsigned char)(((unsigned char)((kd_tmp & 0xF) << 4) & 0xFF) | ((tor_tmp >> 8) & 0xFF));
    _dm4340_mit_command.data[7] = (unsigned char)(tor_tmp & 0xFF);

    return _dm4340_mit_command;
};

can_msgs::Frame DM4340::encodePositionCommand(uint8_t motor_id, float position, float velocity)
{
    uint8_t *pbuf,*vbuf;
    pbuf = (uint8_t*)&position;
    vbuf = (uint8_t*)&velocity;

    _dm4340_position_command.id = 0x100 + motor_id;
    _dm4340_position_command.dlc = 8;
    _dm4340_position_command.data[0] = (unsigned char)(*pbuf);
    _dm4340_position_command.data[1] = (unsigned char)(*(pbuf+1));
    _dm4340_position_command.data[2] = (unsigned char)(*(pbuf+2));
    _dm4340_position_command.data[3] = (unsigned char)(*(pbuf+3));
    _dm4340_position_command.data[4] = (unsigned char)(*vbuf);
    _dm4340_position_command.data[5] = (unsigned char)(*(vbuf+1));
    _dm4340_position_command.data[6] = (unsigned char)(*(vbuf+2));
    _dm4340_position_command.data[7] = (unsigned char)(*(vbuf+3));

    return _dm4340_position_command;
}

can_msgs::Frame DM4340::encodeCalibrateCommand(uint8_t motor_id)
{
    _dm4340_calibrate_command.id = 0x100 + motor_id;
    _dm4340_calibrate_command.dlc = 8;
    _dm4340_calibrate_command.data[0] = 0xFF;
    _dm4340_calibrate_command.data[1] = 0xFF;
    _dm4340_calibrate_command.data[2] = 0xFF;
    _dm4340_calibrate_command.data[3] = 0xFF;
    _dm4340_calibrate_command.data[4] = 0xFF;
    _dm4340_calibrate_command.data[5] = 0xFF;
    _dm4340_calibrate_command.data[6] = 0xFF;
    _dm4340_calibrate_command.data[7] = 0xFE;

    return _dm4340_calibrate_command;
};

can_msgs::Frame DM4340::encodeDisableCommand(uint8_t motor_id)
{
    _dm4340_disable_frame.id = 0x100 + motor_id;
    _dm4340_disable_frame.dlc = 8;
    _dm4340_disable_frame.data[0] = 0xFF;
    _dm4340_disable_frame.data[1] = 0xFF;
    _dm4340_disable_frame.data[2] = 0xFF;
    _dm4340_disable_frame.data[3] = 0xFF;
    _dm4340_disable_frame.data[4] = 0xFF;
    _dm4340_disable_frame.data[5] = 0xFF;
    _dm4340_disable_frame.data[6] = 0xFF;
    _dm4340_disable_frame.data[7] = 0xFD;

    return _dm4340_disable_frame;
};

can_msgs::Frame DM4340::encodeEnableCommand(uint8_t motor_id)
{
    _dm4340_enable_frame.id = 0x100 + motor_id;
    _dm4340_enable_frame.dlc = 8;
    _dm4340_enable_frame.data[0] = 0xFF;
    _dm4340_enable_frame.data[1] = 0xFF;
    _dm4340_enable_frame.data[2] = 0xFF;
    _dm4340_enable_frame.data[3] = 0xFF;
    _dm4340_enable_frame.data[4] = 0xFF;
    _dm4340_enable_frame.data[5] = 0xFF;
    _dm4340_enable_frame.data[6] = 0xFF;
    _dm4340_enable_frame.data[7] = 0xFC;

    return _dm4340_enable_frame;
};

float DM4340::decodePositionFrame(can_msgs::Frame frame)
{
    uint16_t POS = (uint16_t)(frame.data[2] | (frame.data[1] << 8));
    // uint16_t VEL = (uint16_t)(((frame.data[4] & 0xF0) >> 4) | (frame.data[3] << 4));
    // uint16_t TOR = (uint16_t)(frame.data[5] | ((frame.data[4] & 0x0F) << 8));

    _position_state = _uint_to_float(POS, POS_MIN_4340, POS_MAX_4340, 16);
    _position_state = _pi2pi(_position_state);

    return _position_state;
};