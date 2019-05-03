#include "ChassisMotor.h"
#include "can.h"
#include "Encoder.h"

ChassisMotorPID CM_LF_Speedloop;
extern volatile Encoder CM1Encoder;

/**
  *****************************************************************************
  *函数名 ：       底盘电机配置函数
  *函数功能描述 ： 用于配置化电机有关的各项参数 
  *函数参数 ：     无
  *函数返回值 ：   无
  *作者 ：         SLDX->wangX
  *函数创建日期 ： 2018年7月3日
  *函数修改日期 ： 
  *修改人 ：
  *修改原因 ： 
  *版本 ：         1.0.0
  *历史版本 ： 
  ******************************************************************************
  */
void ChassisMotor_Configure()
{
    CM_LF_Speedloop.Kp = 85.0f;
	CM_LF_Speedloop.Ki = 0.0f;
	CM_LF_Speedloop.Kd = 10.0f;
	CM_LF_Speedloop.PoutMax = 5000.0f;
	CM_LF_Speedloop.IoutMax = 0.0f;
	CM_LF_Speedloop.DoutMax = 1000.0f;
	CM_LF_Speedloop.OutMax = 7000.0f;
	CM_LF_Speedloop.Set = 0.0f;
	CM_LF_Speedloop.Real = 0.0f;
	CM_LF_Speedloop.Out = 0.0f;
	CM_LF_Speedloop.err = 0.0f;
	CM_LF_Speedloop.err_last = 0.0f;
	CM_LF_Speedloop.err_llast = 0.0f;
	CM_LF_Speedloop.integral = 0.0f;
}
/**
  *****************************************************************************
  *函数名 ：       地盘电机控制函数
  *函数功能描述 ： 用于pid的实际计算 定时调用，周期4ms 实际值依靠编码器
  *函数参数 ：     无
  *函数返回值 ：   无
  *作者 ：         SLDX->wangX
  *函数创建日期 ： 2018年7月3日
  *函数修改日期 ： 
  *修改人 ：
  *修改原因 ： 
  *版本 ：         1.0.0
  *历史版本 ： 
  ******************************************************************************
  */
void ChassisMotor_Ctrl()
{
    float Pout = 0.0f;
	float Iout = 0.0f;
	float Dout = 0.0f;
	

//  左前电机速度环	
    CM_LF_Speedloop.Real = CM1Encoder.filter_rate;
    
	CM_LF_Speedloop.err_last = CM_LF_Speedloop.err;
	CM_LF_Speedloop.err = CM_LF_Speedloop.Set - CM_LF_Speedloop.Real;
	CM_LF_Speedloop.integral += CM_LF_Speedloop.err;
	
	Pout = CM_LF_Speedloop.Kp * CM_LF_Speedloop.err;
	
    Pout = Pout < CM_LF_Speedloop.PoutMax ? Pout : CM_LF_Speedloop.PoutMax;
	Pout = Pout > -CM_LF_Speedloop.PoutMax ? Pout : -CM_LF_Speedloop.PoutMax;

	Iout = CM_LF_Speedloop.Ki * CM_LF_Speedloop.integral;
	
    Iout = Iout < CM_LF_Speedloop.IoutMax ? Iout : CM_LF_Speedloop.IoutMax;
	Iout = Iout > -CM_LF_Speedloop.IoutMax ? Iout : -CM_LF_Speedloop.IoutMax;

	Dout = CM_LF_Speedloop.Kd * (CM_LF_Speedloop.err - CM_LF_Speedloop.err_last);
	
    Dout = Dout < CM_LF_Speedloop.DoutMax ? Dout : CM_LF_Speedloop.DoutMax;
	Dout = Dout > -CM_LF_Speedloop.DoutMax ? Dout : -CM_LF_Speedloop.DoutMax;
	
	CM_LF_Speedloop.Out = Pout + Iout + Dout;

    CM_LF_Speedloop.Out = CM_LF_Speedloop.Out < CM_LF_Speedloop.OutMax ? CM_LF_Speedloop.Out : CM_LF_Speedloop.OutMax;
	CM_LF_Speedloop.Out = CM_LF_Speedloop.Out > -CM_LF_Speedloop.OutMax ? CM_LF_Speedloop.Out : -CM_LF_Speedloop.OutMax;
	
}
/**
  *****************************************************************************
  *函数名 ：       地盘电机输出函数 
  *函数功能描述 ： 执行pid解算出的数据
  *函数参数 ：     无
  *函数返回值 ：   无
  *作者 ：         SLDX->wangX
  *函数创建日期 ： 2018年7月3日
  *函数修改日期 ： 
  *修改人 ：
  *修改原因 ： 
  *版本 ：         1.0.0
  *历史版本 ：  
  ******************************************************************************
  */
void ChassisMotor_Out()
{
    Set_ChassisMotor_Current(CM_LF_Speedloop.Out, 0, 0, 0);
}
