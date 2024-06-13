#include <utils/utils.h>
#include <stdint.h>

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