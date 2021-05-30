#ifndef _USART2_H_
#define _USART2_H_
#include "sys.h" 

#define  USART2_RS485                        0 
#if USART2_RS485==1
//USART_SendData函数单独使用前后需要开关使能
#define  USART2_RS485_TX_EN_GPIOx        GPIOG
#define  USART2_RS485_TX_EN_Pin     GPIO_Pin_6
#define  USART2_RS485_TX_EN           PGout(6)
#endif

#define  USART2_MAX_TX_LEN		           600 //最大发送缓存字节数，刨去\0 最大-1
#define  USART2_MAX_RX_LEN		           400 //最大接收缓存字节数，刨去\0 最大-1
extern   volatile u16 USART2_RX_STA;           //接收状态标记
extern   u8   USART2_TX_BUF[USART2_MAX_TX_LEN];//发送缓冲区
extern   u8   USART2_RX_BUF[USART2_MAX_RX_LEN];//接收缓冲区
 
void     USART2_Init(u32 botelv);
void     USART2_printf (char *fmt, ...);
void     USART2_Send_Array (u8 *array,u16 num);
#endif

