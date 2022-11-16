#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

#define LIMIT(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))) //�޷�����

typedef struct _PID
{
	float kp, ki, kd;			 // PID����������
	float error, lastError;		 //��ǰ������һ�ε����
	float integral, maxIntegral; //�������ͻ��ֵ��޷�
	float output, maxOutput;	 // PID���������PID����������
	float MAX_volte;			 //�������ѹ
	float target_volte;			 //Ŀ���ѹֵ
} PID;
typedef struct _AnglePID
{
	PID inner;
	PID outer;
	float output;

} AnglePID;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut, float MAX_volte, float init_target);
void PID_SingleCalc(PID *pid, float reference, float feedback);
void PID_AngleCalc(AnglePID *pid, float target_angle, int32_t total_angle, float speed);
float volte_change(float volte, uint8_t type);
#endif
