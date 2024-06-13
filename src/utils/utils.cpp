#include <utils/utils.h>
#include <stdint.h>
#include <math.h>

// 限制数据最大值与最小值
float _limit(float input, float MIN, float MAX)
{
	if(input < MIN)
	{
		return MIN;
	}
	else if(input > MAX)
	{
		return MAX;
	}
	else
	{
		return input;
	}
};

uint16_t _float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

int16_t _float_to_int(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float _uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

float _pi2pi(float rad)
{
    while(rad > M_PI)
    {
        rad -= 2.0 * M_PI;
    }
    while(rad < -M_PI)
    {
        rad += 2.0 * M_PI;
    }
    return rad;
}