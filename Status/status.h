#define ROBOT 1 //ѡ������� 1��ѡ��ϲ���� ����ѡ�������� ����ѡ�ַ�����


#ifndef _STATUS_
#define _STATUS_
#include "sys.h"

//extern char CanBuffClock; // 0 �����ɲ��� 1 �������ɲ���
void freshCanITstatus(void);
void freshUsartITstatus(void);
void freshPIDITstatus(void);
void openStatus(void);
void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);

void setCanClock(void);
void resetCanClock(void);
char getCanClock(void);

#endif
