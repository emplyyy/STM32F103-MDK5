#ifndef _USART3_H_
#define _USART3_H_
#include "sys.h" 

#define  USART3_RS485                        0 
#if USART3_RS485==1
//USART_SendData��������ʹ��ǰ����Ҫ����ʹ��
#define  USART3_RS485_TX_EN_GPIOx        GPIOG
#define  USART3_RS485_TX_EN_Pin     GPIO_Pin_7
#define  USART3_RS485_TX_EN           PGout(7)
#endif

#define  USART3_MAX_TX_LEN		           600 //����ͻ����ֽ�������ȥ\0 ���-1
#define  USART3_MAX_RX_LEN		           400 //�����ջ����ֽ�������ȥ\0 ���-1
extern   volatile u16 USART3_RX_STA;           //����״̬���
extern   u8   USART3_TX_BUF[USART3_MAX_TX_LEN];//���ͻ�����
extern   u8   USART3_RX_BUF[USART3_MAX_RX_LEN];//���ջ�����
 
void     USART3_Init(u32 botelv);
void     USART3_printf (char *fmt, ...);
void     USART3_Send_Array (u8 *array,u16 num);

#endif

