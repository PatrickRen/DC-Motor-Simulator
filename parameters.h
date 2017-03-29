// HysteresisController.h : ��Motor�����ļ�
#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <fstream>
enum Type_E
/*-------------------------------------------------------
	@enum Type_E
	@������ö������ Type_E��������ָ�������
-------------------------------------------------------*/
{
    U_DC, Phi_f, R_q, L_q, J, B, I_h, I_l, Speed_1, Speed_2, Delta
};


class Motor
/*-------------------------------------------------------
	@RKMotorSimu
	@�����������
-------------------------------------------------------*/
{
public:
	Motor();  //���캯�������ڳ�ʼ��
	double GetParameter(Type_E Para);  //��ȡ�����������
private:
	double DCPower;  //�����ѹ
	double MagExcitation;  //d�����
	double ArmatureResistance;  //�������
	double QInductance;  //Q��Ȧ���
	double MomentOfInertia;  //ת��ת��
	double DampingTouqueFactor;  //����ת��ϵ��
	double UpperCurrent;  //��������
	double LowerCurrent;  //��������
	double Speed1;  //ת��1
	double Speed2;  //ת��2
	double DeltaSpeed;  //ת���ͻ����
};
#endif

