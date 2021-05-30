#ifndef _USART1_H_
#define _USART1_H_
#include "sys.h" 

#define  USART1_RS485                        0 
#if USART1_RS485==1
//USART_SendData��������ʹ��ǰ����Ҫ����ʹ��
#define  USART1_RS485_TX_EN_GPIOx        GPIOG
#define  USART1_RS485_TX_EN_Pin     GPIO_Pin_5
#define  USART1_RS485_TX_EN           PGout(5)
#endif

#define  USART1_MAX_TX_LEN		           600 //����ͻ����ֽ�������ȥ\0 ���-1
#define  USART1_MAX_RX_LEN		           400 //�����ջ����ֽ�������ȥ\0 ���-1
extern   volatile u16 USART1_RX_STA;           //����״̬���
extern   u8   USART1_TX_BUF[USART1_MAX_TX_LEN];//���ͻ�����
extern   u8   USART1_RX_BUF[USART1_MAX_RX_LEN];//���ջ�����
 
void     USART1_Init(u32 botelv);
void     USART1_printf (char *fmt, ...);
void     USART1_Send_Array (u8 *array,u16 num);
#endif

