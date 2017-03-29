// RongeKutta.cpp : 定义类RKMotorSimu

#include "stdafx.h"
#include "RongeKutta.h"
#include "parameters.h"

#include <fstream>

extern std::ifstream InitFile;
extern std::ofstream K;
void PassChar(void);

RKMotorSimu::RKMotorSimu(void)
/*------------------------------------------------------------
	@RKMotorSimu(void)
	@功能：类RKMotorSimu的构造函数，从文件中读取电机仿真步长
------------------------------------------------------------*/
{
	PassChar();
	InitFile >> h;
	PassChar();
	InitFile >> time;
	InitFile.close();
	iq1 = 0;
	w1 = 0;
}

void RKMotorSimu::RKCalc(Motor M, double iq0, double w0, bool IGBT)
/*------------------------------------------------------------
	@void RKCalc(Motor M, double iq0, double w0, bool IGBT)
	@功能：对给定的电流预报点iq0、转速预报点w0，对电机M使用四阶
	龙格-库塔法计算下一预报点，存入类成员iq1、w1
	@参数：
	1. Motor M: 电机对象M
	2. iq0: 电流预报点
	3. w0: 转速预报点
	4. IGBT: 功率管开关状态，由电流控制器与转速控制器共同决定
	@电机状态方程（电动机原则）：
	pi_q = (u_q-w*Phi_f-R_q*i_q)/L_q
	pw = (Phi_f*i_q-B*w)/J
------------------------------------------------------------*/
{
	double IGBTState = static_cast<double>(IGBT);  //强制转换成double与Uq进行乘法

	//状态方程参数
	double Uq = M.GetParameter(U_DC);
	double Phif = M.GetParameter(Phi_f);
	double Rq = M.GetParameter(R_q);
	double Lq = M.GetParameter(L_q);
	double BB = M.GetParameter(B);
	double JJ = M.GetParameter(J);

	//四阶龙格-库塔法计算
	double Ki1 = (Uq*IGBTState - w0*Phif - Rq*iq0) / Lq;
	double Kw1 = (Phif*iq0 - BB*w0) / JJ;

	double iq0_51 = iq0 + h / 2 * Ki1;
	double w0_51 = w0 + h / 2 * Kw1;
	double Ki2 = (Uq*IGBTState - w0_51*Phif - Rq*iq0_51) / Lq;
	double Kw2 = (Phif*iq0_51 - BB*w0_51) / JJ;

	double iq0_52 = iq0 + h / 2 * Ki2;
	double w0_52 = w0 + h / 2 * Kw2;
	double Ki3 = (Uq*IGBTState - w0_52*Phif - Rq*iq0_52) / Lq;
	double Kw3 = (Phif*iq0_52 - BB*w0_52) / JJ;

	double iq1_1 = iq0 + h * Ki3;
	double w1_1 = w0 + h * Kw3;
	double Ki4 = (Uq*IGBTState - w1_1*Phif - Rq*iq1_1) / Lq;
	double Kw4 = (Phif*iq1_1 - BB*w1_1) / JJ;

	double Ki = (Ki1 + 2*Ki2 + 2*Ki3 + Ki4)/6.0;
	double Kw = (Kw1 + 2*Kw2 + 2*Kw3 + Kw4)/6.0;

	//K << Ki1 << ' ' << Ki2 << ' ' << Ki3 << ' ' << Ki4 << ' ' << Kw1 << ' ' << Kw2 << ' ' << Kw3 << ' ' << Kw4 << std::endl;

	iq1 = iq0 + h*Ki;
	w1 = w0 + h*Kw;

	if (iq1 < 0)
	{
		if (w1*Phif <= Uq) iq1 = 0;
	}
	//if (iq1 < 0) iq1 = 0;  //如果计算出电流小于0则直接置为0
}

double RKMotorSimu::GetIResult(void)
/*------------------------------------------------------------
	@double GetIResult(void)
	@功能：读取电流计算值iq1
------------------------------------------------------------*/
{
	return iq1;
}

double RKMotorSimu::GetSpeedResult(void)
/*------------------------------------------------------------
	@double GetSpeedResult(void)
	@功能：读取转速计算值w1
------------------------------------------------------------*/
{
	return w1;
}

double RKMotorSimu::GetStep(void)
/*------------------------------------------------------------
	@double GetStep(void)
	@功能：读取步长设定值h
------------------------------------------------------------*/
{
	return h;
}

double RKMotorSimu::GetSimuTime(void)
/*------------------------------------------------------------
	@double GetStep(void)
	@功能：读取仿真时间设定值time
------------------------------------------------------------*/
{
	return time;
}
