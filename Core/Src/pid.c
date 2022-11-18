#include "pid.h"
#include "math.h"
#include "stdlib.h"
// ��ѹ����ת��
float volte_change(float volte, uint8_t type)
{
	// type:0,��3.3-10Vת��Ϊ0-3.3V
	// type:1,��0-3.3Vת��Ϊ3.3-10V
	if (type == 0)
	{
		return (float)(volte - 0.0127f) / 4.5605f;
	}
	else
	{
		return (float)volte * 4.5605f + 0.0127f;
	}
}
// PID��ʼ������
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut, float MAX_volte, float init_target)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxIntegral = maxI;
	pid->maxOutput = maxOut;
	pid->MAX_volte = MAX_volte;
	pid->target_volte = init_target;
}

//����PID���㣬��Ҫ����Ĳ�����PID�ṹ�壬Ŀ��ֵ������ֵ,����λ����PID
void PID_SingleCalc(PID *pid, float reference, float feedback)
{
	static uint16_t inl_index = 0;
	reference = volte_change(reference, 0);
	pid->error = (reference - feedback) * 800.0f / 2.1899f; //���µ�ǰ���
	//����ֱ���P��I��D�ļ���
	pid->output = pid->error * pid->kp; // PΪ���ݵ�ǰ�����������
	if (fabs(pid->error) > 2)
	{
		inl_index = 0;
	}
	else
	{
		inl_index = 1;
	}
	inl_index = 1;
	pid->integral += pid->error * pid->ki; // IΪ�ۼ����������
	// LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);  //����I����������Ƴ���
	pid->output += inl_index * pid->integral;
	pid->output += (pid->error - pid->lastError) * pid->kd; // D�Ե�ǰ����ȥ�ϴ������Ϊ΢�ֻ���
	LIMIT(pid->output, 0, pid->maxOutput);					//����PID�����
	pid->lastError = pid->error;							//������һ�ε����
}

//����PID����
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
