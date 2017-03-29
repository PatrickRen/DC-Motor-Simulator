// RongeKutta.h : 类RKMotorSimu声明文件
#ifndef RONGEKUTTA_H
#define RONGEKUTTA_H
#include "parameters.h"
class RKMotorSimu
/*-------------------------------------------------------
	@RKMotorSimu
	@描述：四阶龙格-库塔法仿真类
-------------------------------------------------------*/
{
public:
	RKMotorSimu(void);  //构造函数，用于初始化
	void RKCalc(Motor M,double iq0, double w0, bool IGBT);  //R-K法计算方法
	double GetIResult(void);  //读取电流方法
	double GetSpeedResult(void);  //读取转速方法
	double GetStep(void);  //读取步长值方法
	double GetSimuTime(void);  //读取仿真时间方法
private:
	double h;  //仿真步长值
	double iq1;  //电流计算值
	double w1;  //转速计算值
	double time;  //仿真总时间
};
#endif