#ifndef _USART2_DMA_H_
#define _USART2_DMA_H_
#include "sys.h" 

#define  USART2_DMA                          1 //����ע���ͻ

#define  USART2_RS485                        1 
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
 
void     USART2_Init(u32 bound);
void     USART2_printf (char *fmt, ...);
void     USART2_Send_Array (u8 *array,u16 num);

#if USART2_DMA==1  
volatile u8   USART2_DMA_Tx_flag;              //DMAæµ��־,1	
volatile u8   USART2_DMA_Rx_flag;              //DMA������ɱ�־,0
void     DMA_USART2_Init(void); 
void     DMA_USART2_Tx_Data(u16 size);
#endif

#endif
