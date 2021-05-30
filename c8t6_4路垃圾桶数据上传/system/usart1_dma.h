#ifndef _USART1_DMA_H_
#define _USART1_DMA_H_
#include "sys.h" 

#define  USART1_DMA                         1 //����ע���ͻ

#define  RS485_USART1                       0 
#if RS485_USART1==1
//USART_SendData��������ʹ��ǰ����Ҫ����ʹ��
#define  RS485_USART1_TX_EN_GPIOx       GPIOG
#define  RS485_USART1_TX_EN_Pin    GPIO_Pin_5
#define  RS485_USART1_TX_EN           PGout(5)
#endif

#define  USART1_MAX_TX_LEN		           600 //����ͻ����ֽ�������ȥ\0 ���-1
#define  USART1_MAX_RX_LEN		           400 //�����ջ����ֽ�������ȥ\0 ���-1
extern   volatile u16 USART1_RX_STA;           //����״̬���
extern   u8   USART1_TX_BUF[USART1_MAX_TX_LEN];//���ͻ�����
extern   u8   USART1_RX_BUF[USART1_MAX_RX_LEN];//���ջ�����    		             
 
void     USART1_Init(u32 bound);
void     USART1_printf (char *fmt, ...);
void     USART1_Send_Array (u8 *array,u16 num);
void     uartScreen(u8 a, u8 b, u8 c, u8 d);

#if USART1_DMA==1  
volatile u8   USART1_DMA_Tx_flag;              //DMAæµ��־,1	
volatile u8   USART1_DMA_Rx_flag;              //DMA������ɱ�־,0
void     DMA_USART1_Init(void); 
void     DMA_USART1_Tx_Data(u16 size);
#endif

#endif
