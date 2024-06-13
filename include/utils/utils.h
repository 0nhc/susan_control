#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

// 使用Union来转换32bit数据的不同写法
union Convert32
{
	float to_float;
	int to_int;
	int32_t to_int32;
	uint32_t to_uint32;
	uint8_t data[4];
};

// 使用Union来转换16bit数据的不同写法
union Convert16
{
	int16_t to_int16;
    uint16_t to_uint16;
	uint8_t data[2];
};

// 限制数据最大值与最小值
float _limit(float input, float MIN, float MAX);

#endif