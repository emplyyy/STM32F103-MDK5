#include "UltrasonicWave.h"
#include "delay.h"


#define	TRIG_PORT      GPIOA		//TRIG       
//#define	ECHO_PORT      GPIOC		//ECHO 
//#define	TRIG_PIN       GPIO_Pin_8   //TRIG       
//#define	ECHO_PIN       GPIO_Pin_7	//ECHO    

//float UltrasonicWave_Distance;      //计算出的距离    
//u8 paddr[10];

/*
 * 函数名：UltrasonicWave_Configuration
 * 描述  ：超声波模块的初始化
 * 输入  ：无
 * 输出  ：无	
 */
void UltrasonicWave_Configuration(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 
 GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);						 

}
/* 工作原理：PC8接TRIG并设为推挽输出模式，PC8发出大于10us高电平后，超声波遇到障碍会
返回波形给PC7接收；超声波发送器模块是超声波源，只要控制PC8端口就能控制超声波源；
PC7接ECH0并设为下拉输入；PC7设置为外部中断：上升沿触发
（当PC7接收到回波时PC7会由下拉变为高电平，从而触发外部中断）；当PC7变为上升沿时
即触发外部中断，进入中断服务函数：将TIM4->CNT清零，TIM4开始计数，当PC7变为下降沿时
TIM4停止计数，计算得出UltrasonicWave_Distance的值；清除EXTI7线路挂起位；

将PC7输入到TIM4的任意捕获端口，再由捕获程序计算出距离
*/

//void EXTI9_5_IRQHandler(void)
//{
//	delay_us(10);		                      //延时10us
//  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
//	{
//			TIM_SetCounter(TIM4,0);
//			TIM_Cmd(TIM4, ENABLE);      //开启时钟
//		
//			while(GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN));	//等待低电平，如果为高电平就一直计数

//			TIM_Cmd(TIM4, DISABLE);  //定时器2失能
//		//	UltrasonicWave_Distance=TIM_GetCounter(TIM4)*5*34/200.0;		//计算距离&&UltrasonicWave_Distance<150
//		UltrasonicWave_Distance=TIM_GetCounter(TIM4)*17/1000;//计算距离&&UltrasonicWave_Distance<85
//	  printf("distance:%f cm",UltrasonicWave_Distance);
//		if(UltrasonicWave_Distance>0)
//	{  sprintf((char*)paddr, "Distance:%1f", UltrasonicWave_Distance);
//		//	LCD_DispNumber(80, 120, UltrasonicWave_Distance, 0);
//	  	LCD_DispStr(10,100, paddr, BLUE);//显示p的地址
//		//printf("distance:%f cm",UltrasonicWave_Distance);
//	}
//		
//	
//	
//	EXTI_ClearITPendingBit(EXTI_Line7);  //清除EXTI7线路挂起位
//}

//}

/*
 * 函数名：UltrasonicWave_StartMeasure
 * 描述  ：开始测距，发送一个>10us的脉冲，然后测量返回的高电平时间
 * 输入  ：无
 * 输出  ：无	
 */
void UltrasonicWave_StartMeasure(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_4);  //送>10US的高电平RIG_PORT,TRIG_PIN这两个在define中有?
  delay_us(20);		                      //延时20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_4);
 
//	GPIO_SetBits(TRIG_PORT,GPIO_Pin_6);  //送>10US的高电平RIG_PORT,TRIG_PIN这两个在define中有?
//  delay_us(20);		                      //延时20US
//  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_6);
}
void UltrasonicWave_StartMeasure2(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_5);  //送>10US的高电平RIG_PORT,TRIG_PIN这两个在define中有?
  delay_us(20);		                      //延时20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_5);
}
void UltrasonicWave_StartMeasure3(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_6);  //送>10US的高电平RIG_PORT,TRIG_PIN这两个在define中有?
  delay_us(20);		                      //延时20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_6);
}

void UltrasonicWave_StartMeasure4(void)
{
  GPIO_SetBits(TRIG_PORT,GPIO_Pin_7);  //送>10US的高电平RIG_PORT,TRIG_PIN这两个在define中有?
  delay_us(20);		                      //延时20US
  GPIO_ResetBits(TRIG_PORT,GPIO_Pin_7);
}

/******************* (C) 1209 Lab *****END OF FILE************/
