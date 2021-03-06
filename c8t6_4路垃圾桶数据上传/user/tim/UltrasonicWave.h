#ifndef __UltrasonicWave_H
#define	__UltrasonicWave_H

#include "stm32f10x.h"

void UltrasonicWave_Configuration(void);               //对超声波模块初始化
void UltrasonicWave_StartMeasure(void);                //开始测距，发送一个>10us的脉冲，然后测量返回的高电平时间
void UltrasonicWave_StartMeasure2(void);
void UltrasonicWave_StartMeasure3(void);
void UltrasonicWave_StartMeasure4(void);
#endif /* __UltrasonicWave_H */
