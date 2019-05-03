#ifndef _CLOUDMOTOR_
#define _CLOUDMOTOR_

typedef struct CloudMotorPID 
{
	float Kp,Ki,Kd;    //定义 
	float PoutMax,IoutMax,DoutMax;
	float OutMax;
	
	float Set;    //定义设定值
	float Real; //编码器采样值
    float Out; //输出值
	
    float err;         //定义偏差值
    float err_last; //上一次偏差值
    float err_llast; //最上次偏差值
	float integral; //误差累计
}CloudMotorPID;
void CloudMotor_Configure(void);
void CloudMotor_Pitch_Ctrl(void);
void CloudMotor_Out(void);


#endif
