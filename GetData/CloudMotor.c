#include "CloudMotor.h"
#include "Encoder.h"
#include "can.h"
#include "mpu6050.h"
CloudMotorPID CM_Pitch_Inloop;
CloudMotorPID CM_Pitch_Outloop;

extern volatile Encoder GMPitchEncoder;

/**
  *****************************************************************************
  *函数名 ： 
  *函数功能描述 ： 
  *函数参数 ： 
  *函数返回值 ： 
  *作者 ：
  *函数创建日期 ： 
  *函数修改日期 ： 
  *修改人 ：
  *修改原因 ： 
  *版本 ： 
  *历史版本 ： 
  ******************************************************************************
  */
void CloudMotor_Configure()
{
	//Pitch轴电机参数外环
	CM_Pitch_Outloop.Kp = 0.0f;
	CM_Pitch_Outloop.Ki = 0.0f;
	CM_Pitch_Outloop.Kd = 0.0f;
	CM_Pitch_Outloop.PoutMax = 0.0f;
	CM_Pitch_Outloop.IoutMax = 0.0f;
	CM_Pitch_Outloop.DoutMax = 0.0f;
	CM_Pitch_Outloop.OutMax = 0.0f;
	CM_Pitch_Outloop.Set = 0.0f;
	CM_Pitch_Outloop.Real = 0.0f;
	CM_Pitch_Outloop.Out = 0.0f;
	CM_Pitch_Outloop.err = 0.0f;
	CM_Pitch_Outloop.err_last = 0.0f;
	CM_Pitch_Outloop.err_llast = 0.0f;
	CM_Pitch_Outloop.integral = 0.0f;
	//Pitch轴电机参数内环
	CM_Pitch_Inloop.Kp = 70.0f;
	CM_Pitch_Inloop.Ki = 0.0f;
	CM_Pitch_Inloop.Kd = 5.0f;
	CM_Pitch_Inloop.PoutMax = 1200.0f;
	CM_Pitch_Inloop.IoutMax = 0.0f;
	CM_Pitch_Inloop.DoutMax = 200.0f;
	CM_Pitch_Inloop.OutMax = 1200.0f;
	CM_Pitch_Inloop.Set = 0.0f;
	CM_Pitch_Inloop.Real = 0.0f;
	CM_Pitch_Inloop.Out = 0.0f;
	CM_Pitch_Inloop.err = 0.0f;
	CM_Pitch_Inloop.err_last = 0.0f;
	CM_Pitch_Inloop.err_llast = 0.0f;
	CM_Pitch_Inloop.integral = 0.0f;
}
/**
  *****************************************************************************
  *函数名 ： 
  *函数功能描述 ： 
  *函数参数 ： 
  *函数返回值 ： 
  *作者 ：
  *函数创建日期 ： 
  *函数修改日期 ： 
  *修改人 ：
  *修改原因 ： 
  *版本 ： 
  *历史版本 ： 
  ******************************************************************************
  */
float x,z;
void CloudMotor_Pitch_Ctrl()
{
	float Pout = 0.0f;
	float Iout = 0.0f;
	float Dout = 0.0f;
	
//	read_Gyrodate(&x, &z);
//  外环计算	
	CM_Pitch_Outloop.err_last = CM_Pitch_Outloop.err;
	CM_Pitch_Outloop.err = CM_Pitch_Outloop.Set - CM_Pitch_Outloop.Real;
	CM_Pitch_Outloop.integral += CM_Pitch_Outloop.err;
	
	Pout = CM_Pitch_Outloop.Kp * CM_Pitch_Outloop.err;
	Pout = Pout < CM_Pitch_Outloop.PoutMax ? Pout : CM_Pitch_Outloop.PoutMax;
	Pout = Pout > -CM_Pitch_Outloop.PoutMax ? Pout : -CM_Pitch_Outloop.PoutMax;

	Iout = CM_Pitch_Outloop.Ki * CM_Pitch_Outloop.integral;
	Iout = Iout < CM_Pitch_Outloop.IoutMax ? Iout : CM_Pitch_Outloop.IoutMax;
	Iout = Iout > -CM_Pitch_Outloop.IoutMax ? Iout : -CM_Pitch_Outloop.IoutMax;

	Dout = CM_Pitch_Outloop.Kd * (CM_Pitch_Outloop.err - CM_Pitch_Outloop.err_last);
	Dout = Dout < CM_Pitch_Outloop.DoutMax ? Dout : CM_Pitch_Outloop.DoutMax;
	Dout = Dout > -CM_Pitch_Outloop.DoutMax ? Dout : -CM_Pitch_Outloop.DoutMax;
	
	CM_Pitch_Outloop.Out = Pout + Iout + Dout;
	CM_Pitch_Outloop.Out = CM_Pitch_Outloop.Out < CM_Pitch_Outloop.OutMax ? CM_Pitch_Outloop.Out : CM_Pitch_Outloop.OutMax;
	CM_Pitch_Outloop.Out = CM_Pitch_Outloop.Out > -CM_Pitch_Outloop.OutMax ? CM_Pitch_Outloop.Out : -CM_Pitch_Outloop.OutMax;
	
//  内环计算
//	CM_Pitch_Inloop.Set = CM_Pitch_Outloop.Out;
	CM_Pitch_Inloop.Real = GMPitchEncoder.ecd_angle;

	CM_Pitch_Inloop.err_last = CM_Pitch_Inloop.err;
	CM_Pitch_Inloop.err = CM_Pitch_Inloop.Set - CM_Pitch_Inloop.Real;
	CM_Pitch_Inloop.integral += CM_Pitch_Inloop.err;
	
	Pout = CM_Pitch_Inloop.Kp * CM_Pitch_Inloop.err;
	Pout = Pout < CM_Pitch_Inloop.PoutMax ? Pout : CM_Pitch_Inloop.PoutMax;
	Pout = Pout > -CM_Pitch_Inloop.PoutMax ? Pout : -CM_Pitch_Inloop.PoutMax;

	Iout = CM_Pitch_Inloop.Ki * CM_Pitch_Inloop.integral;
	Iout = Iout < CM_Pitch_Inloop.IoutMax ? Iout : CM_Pitch_Inloop.IoutMax;
	Iout = Iout > -CM_Pitch_Inloop.IoutMax ? Iout : -CM_Pitch_Inloop.IoutMax;

	Dout = CM_Pitch_Inloop.Kd * (CM_Pitch_Inloop.err - CM_Pitch_Inloop.err_last);
	Dout = Dout < CM_Pitch_Inloop.DoutMax ? Dout : CM_Pitch_Inloop.DoutMax;
	Dout = Dout > -CM_Pitch_Inloop.DoutMax ? Dout : -CM_Pitch_Inloop.DoutMax;
	
	CM_Pitch_Inloop.Out = Pout + Iout + Dout;
	CM_Pitch_Inloop.Out = CM_Pitch_Inloop.Out < CM_Pitch_Inloop.OutMax ? CM_Pitch_Inloop.Out : CM_Pitch_Inloop.OutMax;
	CM_Pitch_Inloop.Out = CM_Pitch_Inloop.Out > -CM_Pitch_Inloop.OutMax ? CM_Pitch_Inloop.Out : -CM_Pitch_Inloop.OutMax;	
}
/**
  *****************************************************************************
  *函数名 ： 
  *函数功能描述 ： 
  *函数参数 ： 
  *函数返回值 ： 
  *作者 ：
  *函数创建日期 ： 
  *函数修改日期 ： 
  *修改人 ：
  *修改原因 ： 
  *版本 ： 
  *历史版本 ： 
  ******************************************************************************
  */
void CloudMotor_Out()
{
	Set_CloudMotor_Current(0, -(int16_t)CM_Pitch_Inloop.Out);
}
