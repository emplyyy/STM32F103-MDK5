#ifndef __UltrasonicWave_H
#define	__UltrasonicWave_H

#include "stm32f10x.h"

void UltrasonicWave_Configuration(void);               //�Գ�����ģ���ʼ��
void UltrasonicWave_StartMeasure(void);                //��ʼ��࣬����һ��>10us�����壬Ȼ��������صĸߵ�ƽʱ��
void UltrasonicWave_StartMeasure2(void);
void UltrasonicWave_StartMeasure3(void);
void UltrasonicWave_StartMeasure4(void);
#endif /* __UltrasonicWave_H */
