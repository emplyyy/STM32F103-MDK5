#ifndef _USART2_H_
#define _USART2_H_
#include "sys.h" 

#define  USART2_RS485                        0 
#if USART2_RS485==1
//USART_SendData��������ʹ��ǰ����Ҫ����ʹ��
#define  USART2_RS485_TX_EN_GPIOx        GPIOG
#define  USART2_RS485_TX_EN_Pin     GPIO_Pin_6
#define  USART2_RS485_TX_EN           PGout(6)
#endif

#define  USART2_MAX_TX_LEN		           600 //����ͻ����ֽ�������ȥ\0 ���-1
#define  USART2_MAX_RX_LEN		           400 //�����ջ����ֽ�������ȥ\0 ���-1
extern   volatile u16 USART2_RX_STA;           //����״̬���
extern   u8   USART2_TX_BUF[USART2_MAX_TX_LEN];//���ͻ�����
extern   u8   USART2_RX_BUF[USART2_MAX_RX_LEN];//���ջ�����
 
void     USART2_Init(u32 botelv);
void     USART2_printf (char *fmt, ...);
void     USART2_Send_Array (u8 *array,u16 num);
#endif

