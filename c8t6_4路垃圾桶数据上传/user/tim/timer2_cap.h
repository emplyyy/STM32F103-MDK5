#ifndef __TIMER2_CAP_H
#define __TIMER2_CAP_H


#include "stm32f10x.h"

extern u32 tempup1;	//捕获总高电平的时间
extern u32 tempup2;	//捕获总高电平的时间
extern u32 tempup3;	//捕获总高电平的时间
extern u32 tempup4;	//捕获总高电平的时间

void TIM2_Cap_Init(u16 arr, u16 psc);

#endif
