// PIDController.h : ��PIDCtrler�����ļ�
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "parameters.h"
#include "RongeKutta.h"
class PIDCtrler
/*-------------------------------------------------------
	@PIDCtrler
	@������PID��������
-------------------------------------------------------*/
{
public:
	PIDCtrler(Motor M, RKMotorSimu S);  //���캯�������ڳ�ʼ��
	void SetGain(double Kp, double Ki, double Kd);  //����PID���������淽��
	void SetSpeed(double Speed);  //�趨ת��ֵ����
	double PIDCalc(double SpeedNow);  //PID����������PWMռ�ձȷ���
private:
	double Gain_P;  //�����ؼ�����
	double Gain_I;  //���ֿؼ�����
	double Gain_D;  //΢�ֿؼ�����
	double SpeedSet;  //ת���趨ֵ
	double PreError;  //����ۻ�
	double Integral;  //������
	double dt;  //ʱ��΢��Ԫ
};

class PWMCtrler
/*-------------------------------------------------------
	@PIDCtrler
	@������PWM��������
-------------------------------------------------------*/
{
public:
	PWMCtrler(RKMotorSimu S);  //���캯�������ڳ�ʼ��
	bool JudgeState(double TimeNow);  //״̬�жϷ���
	void SetDutyCycle(double d);  //�趨ռ�ձȷ���
private:
	bool State;  //PWM״̬
	double DutyCycle;  //ռ�ձ�
	double Period;  //PWM����
};
#endif