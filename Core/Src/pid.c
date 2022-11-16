#include "pid.h"
#include "math.h"
#include "stdlib.h"
// 电压倍数转换
float volte_change(float volte, uint8_t type)
{
	// type:0,把3.3-10V转换为0-3.3V
	// type:1,把0-3.3V转换为3.3-10V
	if (type == 0)
	{
		return (float)volte * (43.0 / 105 - 22.0 / 121);
	}
	else
	{
		return (float)volte / (43.0 / 105 - 22.0 / 121);
	}
}
// PID初始化函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut, float MAX_volte)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxIntegral = maxI;
	pid->maxOutput = maxOut;
	pid->MAX_volte = MAX_volte;
	pid->target_volte = 0;
}

//单级PID计算，需要传入的参数，PID结构体，目标值，反馈值,采用位置型PID
void PID_SingleCalc(PID *pid, float reference, float feedback)
{
	static uint16_t inl_index = 0;
	reference = volte_change(reference, 0);
	pid->error = (reference - feedback) * 800.0f / 2.27f; //更新当前误差
	//下面分别是P，I，D的计算
	pid->output = pid->error * pid->kp; // P为根据当前误差计算输出量
	if (fabs(pid->error) > 2)
	{
		inl_index = 0;
	}
	else
	{
		inl_index = 1;
	}
	inl_index = 1;
	pid->integral += pid->error * pid->ki; // I为累计误差的输出量
	// LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);  //限制I的输出，抑制超调
	pid->output += inl_index * pid->integral;
	pid->output += (pid->error - pid->lastError) * pid->kd; // D以当前误差减去上次误差作为微分环节
	LIMIT(pid->output, 0, pid->maxOutput);					//限制PID总输出
	pid->lastError = pid->error;							//更新上一次的误差
}

//串级PID计算
void PID_AngleCalc(AnglePID *pid, float target_angle, int32_t total_angle, float speed)
{
	PID_SingleCalc(&pid->outer, target_angle, total_angle);
	if (pid->outer.output > pid->outer.MAX_volte)
	{
		pid->outer.output = pid->outer.MAX_volte;
	}
	else if (pid->outer.output < -pid->outer.MAX_volte)
	{
		pid->outer.output = -pid->outer.MAX_volte;
	}
	PID_SingleCalc(&pid->inner, pid->outer.output, speed);
	pid->output = pid->inner.output;
}
