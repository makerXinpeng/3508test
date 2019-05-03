#define ROBOT 1 //选择机器人 1号选手喜羊羊 二号选手懒羊羊 三号选手沸羊羊


#ifndef _STATUS_
#define _STATUS_
#include "sys.h"

//extern char CanBuffClock; // 0 解锁可操作 1 上锁不可操作
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
