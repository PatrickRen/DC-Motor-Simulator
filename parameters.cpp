// parameters.cpp : ������Motor
#include "stdafx.h"
#include "parameters.h"
#include <fstream>
#include <iostream>
#include <string>
using namespace std;
extern ifstream InitFile;

void PassChar(void)
/*------------------------------------------------------------
	@void PassChar(void)
	@���ܣ����ļ��ж�ȡ�������ʱ�����˵��ַ����Ա�֤������ȷ����
------------------------------------------------------------*/
{
	char c;
	do
	{
		InitFile >> c;
	} while (c != '=');
	//InitFile >> c;
	return;
}

Motor::Motor(void)
/*------------------------------------------------------------
	@Motor(void)
	@���ܣ���Motor�Ĺ��캯�������ļ��ж�ȡ�������
------------------------------------------------------------*/
{
	PassChar();
	InitFile >> DCPower;
	PassChar();
	InitFile >> MagExcitation;
	PassChar();
	InitFile >> ArmatureResistance;
	PassChar();
	InitFile >> QInductance;
	PassChar();
	InitFile >> MomentOfInertia;
	PassChar();
	InitFile >> DampingTouqueFactor;
	PassChar();
	InitFile >> UpperCurrent;
	PassChar();
	InitFile >> LowerCurrent;
	PassChar();
	InitFile >> Speed1;
	PassChar();
	InitFile >> Speed2;
	PassChar();
	InitFile >> DeltaSpeed;
}

double Motor::GetParameter(Type_E Para)
/*------------------------------------------------------------
	@double GetParameter(Type_E Para)
	@���ܣ���Ҫ���ȡ�������
	@������
	1. Type_E Para: ����ȡ�ĵ������
------------------------------------------------------------*/
{
	switch (Para)
	{
		case U_DC: return DCPower;
		case Phi_f: return MagExcitation;
		case R_q: return ArmatureResistance;
		case L_q: return QInductance;
		case J: return MomentOfInertia;
		case B: return DampingTouqueFactor;
		case I_h: return UpperCurrent;
		case I_l: return LowerCurrent;
		case Speed_1: return Speed1;
		case Speed_2: return Speed2;
		case Delta: return DeltaSpeed;
		default: return -1;
	}
}