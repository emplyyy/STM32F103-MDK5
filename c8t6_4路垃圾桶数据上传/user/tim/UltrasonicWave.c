#include "UltrasonicWave.h"
#include "delay.h"


#define	TRIG_PORT      GPIOA		//TRIG       
//#define	ECHO_PORT      GPIOC		//ECHO 
//#define	TRIG_PIN       GPIO_Pin_8   //TRIG       
//#define	ECHO_PIN       GPIO_Pin_7	//ECHO    

//float UltrasonicWave_Distance;      //������ľ���    
//u8 paddr[10];

/*
 * ��������UltrasonicWave_Configuration
 * ����  ��������ģ��ĳ�ʼ��
 * ����  ����
 * ���  ����	
 */
void UltrasonicWave_Configuration(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 
 GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);						 

}
/* ����ԭ��PC8��TRIG����Ϊ�������ģʽ��PC8��������10us�ߵ�ƽ�󣬳����������ϰ���
���ز��θ�PC7���գ�������������ģ���ǳ�����Դ��ֻҪ����PC8�˿ھ��ܿ��Ƴ�����Դ��
PC7��ECH0����Ϊ�������룻PC7����Ϊ�ⲿ�жϣ������ش���
����PC7���յ��ز�ʱPC7����������Ϊ�ߵ�ƽ���Ӷ������ⲿ�жϣ�����PC7��Ϊ������ʱ
�������ⲿ�жϣ������жϷ���������TIM4->CNT���㣬TIM4��ʼ��������PC7��Ϊ�½���ʱ
TIM4ֹͣ����������ó�UltrasonicWave_Distance��ֵ�����EXTI7��·����λ��

��PC7���뵽TIM4�����Ⲷ��˿ڣ����ɲ��������������
*/

//void EXTI9_5_IRQHandler(void)
//{
//	delay_us(10);		                      //��ʱ10us
//  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
//	{
//			TIM_SetCounter(TIM4,0);
//			TIM_Cmd(TIM4, ENABLE);      //����ʱ��
//		
//			while(GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN));	//�ȴ��͵�ƽ�����Ϊ�ߵ�ƽ��һֱ����

//			TIM_Cmd(TIM4, DISABLE);  //��ʱ��2ʧ��
//		//	UltrasonicWave_Distance=TIM_GetCounter(TIM4)*5*34/200.0;		//�������&&UltrasonicWave_Distance<150
//		UltrasonicWave_Distance=TIM_GetCounter(TIM4)*17/1000;//�������&&UltrasonicWave_Distance<85
//	  printf("distance:%f cm",UltrasonicWave_Distance);
//		if(UltrasonicWave_Distance>0)
//	{  sprintf((char*)paddr, "Distance:%1f", UltrasonicWave_Distance);
//		//	LCD_DispNumber(80, 120, UltrasonicWave_Distance, 0);
//	  	LCD_DispStr(10,100, paddr, BLUE);//��ʾp�ĵ�ַ
//		//printf("distance:%f cm",UltrasonicWave_Distance);
//	}
//		
//	
//	
//	EXTI_ClearITPendingBit(EXTI_Line7);  //���EXTI7��·����λ
//}

//}

/*
 * ��������UltrasonicWave_StartMeasure
 * ����  ����ʼ��࣬����һ��>10us�����壬Ȼ��������صĸߵ�ƽʱ��
 * ����  ����
 * ���  ����	
 */
void UltrasonicWave_StartMeasure(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_4);  //��>10US�ĸߵ�ƽ�TRIG_PORT,TRIG_PIN��������define����?
  delay_us(20);		                      //��ʱ20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_4);
 
//	GPIO_SetBits(TRIG_PORT,GPIO_Pin_6);  //��>10US�ĸߵ�ƽ�TRIG_PORT,TRIG_PIN��������define����?
//  delay_us(20);		                      //��ʱ20US
//  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_6);
}
void UltrasonicWave_StartMeasure2(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_5);  //��>10US�ĸߵ�ƽ�TRIG_PORT,TRIG_PIN��������define����?
  delay_us(20);		                      //��ʱ20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_5);
}
void UltrasonicWave_StartMeasure3(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_6);  //��>10US�ĸߵ�ƽ�TRIG_PORT,TRIG_PIN��������define����?
  delay_us(20);		                      //��ʱ20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_6);
}

void UltrasonicWave_StartMeasure4(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_7);  //��>10US�ĸߵ�ƽ�TRIG_PORT,TRIG_PIN��������define����?
  delay_us(20);		                      //��ʱ20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_7);
}

/******************* (C) 1209 Lab *****END OF FILE************/
