#include "status.h"
#include "inv_mpu.h"
#include "can.h"
char CanBuffClock = 0;
u32 CanITstatus = 0, UsartITstatus = 0, PIDITstatus = 0;
//��״̬����1 δ��״̬0
u32 lastC = 1,lastU = 1,lastP = 1;

extern u8 Can_buf_S[8];
extern u8 Can_buf_s_cm[8];

void openStatus(void)
{
	Can_buf_S[0] = 0;Can_buf_S[1] = 0;
	Can_buf_S[2] = 0;Can_buf_S[3] = 0;
	CAN1_Send_PZT_Msg(Can_buf_S,8);
	Can_buf_s_cm[0] = 0;Can_buf_s_cm[1] = 0;
	Can_buf_s_cm[2] = 0;Can_buf_s_cm[3] = 0;
	Can_buf_s_cm[4] = 0;Can_buf_s_cm[5] = 0;
	Can_buf_s_cm[6] = 0;Can_buf_s_cm[7] = 0;
	CAN1_Send_Msg(Can_buf_s_cm,8);
}

//void TIM8_TRG_COM_TIM14_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM14,TIM_IT_Update) == SET)
//	{
//		if(CanITstatus == lastC)
//		{
//			mpu_dmp_init();
//			Can_buf_S[0] = 0;Can_buf_S[1] = 0;
//			Can_buf_S[2] = 0;Can_buf_S[3] = 0;
//			CAN1_Send_PZT_Msg(Can_buf_S,8);
//			Can_buf_s_cm[0] = 0;Can_buf_s_cm[1] = 0;
//			Can_buf_s_cm[2] = 0;Can_buf_s_cm[3] = 0;
//			Can_buf_s_cm[4] = 0;Can_buf_s_cm[5] = 0;
//			Can_buf_s_cm[6] = 0;Can_buf_s_cm[7] = 0;
//			CAN1_Send_Msg(Can_buf_s_cm,8);
//		}	
//	}
//	TIM_ClearITPendingBit(TIM14,TIM_IT_Update);
//}	
void freshCanITstatus(void)
{
	if(CanITstatus < 100)
		CanITstatus ++;
	else
		CanITstatus = 0;
}
void freshUsartITstatus(void)
{
	if(UsartITstatus < 100)
		UsartITstatus ++;
	else
		UsartITstatus = 0;
}
void freshPIDITstatus(void)
{
	if(PIDITstatus < 100)
		PIDITstatus ++;
	else
		PIDITstatus = 0;
}
char getCanClock(void)
{
	return CanBuffClock;
}

void setCanClock(void)
{
	CanBuffClock  = 1;
}

void resetCanClock(void)
{
	CanBuffClock  = 0;
}

//��ʼ���������Ź�
//prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)
//rlr:�Զ���װ��ֵ,0~0XFFF.
//��Ƶ����=4*2^prer.�����ֵֻ����256!
//rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶ�IWDG->PR IWDG->RLR��д
	
	IWDG_SetPrescaler(prer); //����IWDG��Ƶϵ��

	IWDG_SetReload(rlr);   //����IWDGװ��ֵ

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //ʹ�ܿ��Ź�
}

//ι�������Ź�
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}
