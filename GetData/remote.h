/******************************************************************************
 * @file     remote.h
 * @brief    RMң�������ն˳���
 * @author   Liyi
 * @version  V1.1
 * @date     2017.3.10
 * @note
 * @par				��ʼ������Remote_uart_init   ��ȡ���ݺ���GetRemoteData
 *
 * @version  V1.0
 * @date     2017.6.12
 * @note
 * @par				//���ڼ����������PDF�ĵ�
 *
 * Copyright (C) 2017 SLDX Limited. All rights reserved.
 *
 ******************************************************************************/

#ifndef _REMOTE_H
#define _REMOTE_H
#include "stm32f4xx.h" 

typedef struct
{
	int16_t R_x;  //����x		��1684 ��364 ��1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //����y    ��1684 ��364 ��1024
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			//������  ��1 ��3 ��2
	uint8_t sr;
	
	int16_t mouse_x;	//���
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//��갴��
	uint8_t mouse_r;
	uint16_t key;			//����
}RC;
extern RC rc;
void Remote_uart2_init(void);	//ң������ʼ��
//RC GetRemoteData(void);	//����RC�ṹ�����ͱ���

#endif
