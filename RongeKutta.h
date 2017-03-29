// RongeKutta.h : ��RKMotorSimu�����ļ�
#ifndef RONGEKUTTA_H
#define RONGEKUTTA_H
#include "parameters.h"
class RKMotorSimu
/*-------------------------------------------------------
	@RKMotorSimu
	@�������Ľ�����-������������
-------------------------------------------------------*/
{
public:
	RKMotorSimu(void);  //���캯�������ڳ�ʼ��
	void RKCalc(Motor M,double iq0, double w0, bool IGBT);  //R-K�����㷽��
	double GetIResult(void);  //��ȡ��������
	double GetSpeedResult(void);  //��ȡת�ٷ���
	double GetStep(void);  //��ȡ����ֵ����
	double GetSimuTime(void);  //��ȡ����ʱ�䷽��
private:
	double h;  //���沽��ֵ
	double iq1;  //��������ֵ
	double w1;  //ת�ټ���ֵ
	double time;  //������ʱ��
};
#endif