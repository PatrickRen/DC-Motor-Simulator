// HysteresisController.h : 类Motor声明文件
#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <fstream>
enum Type_E
/*-------------------------------------------------------
	@enum Type_E
	@描述：枚举类型 Type_E，用来代指电机参数
-------------------------------------------------------*/
{
    U_DC, Phi_f, R_q, L_q, J, B, I_h, I_l, Speed_1, Speed_2, Delta
};


class Motor
/*-------------------------------------------------------
	@RKMotorSimu
	@描述：电机类
-------------------------------------------------------*/
{
public:
	Motor();  //构造函数，用于初始化
	double GetParameter(Type_E Para);  //读取电机参数方法
private:
	double DCPower;  //电机电压
	double MagExcitation;  //d轴磁链
	double ArmatureResistance;  //电枢电阻
	double QInductance;  //Q线圈电感
	double MomentOfInertia;  //转子转矩
	double DampingTouqueFactor;  //阻尼转矩系数
	double UpperCurrent;  //电流上限
	double LowerCurrent;  //电流下限
	double Speed1;  //转速1
	double Speed2;  //转速2
	double DeltaSpeed;  //转速滞环宽度
};
#endif

