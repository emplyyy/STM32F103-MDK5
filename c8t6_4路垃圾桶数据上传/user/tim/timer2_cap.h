#ifndef __TIMER2_CAP_H
#define __TIMER2_CAP_H


#include "stm32f10x.h"

extern u32 tempup1;	//�����ܸߵ�ƽ��ʱ��
extern u32 tempup2;	//�����ܸߵ�ƽ��ʱ��
extern u32 tempup3;	//�����ܸߵ�ƽ��ʱ��
extern u32 tempup4;	//�����ܸߵ�ƽ��ʱ��

void TIM2_Cap_Init(u16 arr, u16 psc);

#endif
