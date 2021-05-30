#ifndef _USART3_DMA_H_
#define _USART3_DMA_H_
#include "sys.h" 

#define  USART3_DMA                          1 //开启注意冲突

#define  USART3_RS485                        0 
#if USART3_RS485==1
//USART_SendData函数单独使用前后需要开关使能
#define  USART3_RS485_TX_EN_GPIOx        GPIOG
#define  USART3_RS485_TX_EN_Pin     GPIO_Pin_7
#define  USART3_RS485_TX_EN           PGout(7)
#endif

#define  USART3_MAX_TX_LEN		           600 //最大发送缓存字节数，刨去\0 最大-1
#define  USART3_MAX_RX_LEN		           400 //最大接收缓存字节数，刨去\0 最大-1
extern   volatile u16 USART3_RX_STA;           //接收状态标记
extern   u8   USART3_TX_BUF[USART3_MAX_TX_LEN];//发送缓冲区
extern   u8   USART3_RX_BUF[USART3_MAX_RX_LEN];//接收缓冲区    		             
 
void     USART3_Init(u32 bound);
void     USART3_printf (char *fmt, ...);
void     USART3_Send_Array (u8 *array,u16 num);
int      NBiot_SendCmd(char* cmd, char* reply, int wait);
int      NBiot_Init(void);
int      Get_IP(void);
void     connected_IP(char* IP,char*Port,char* ID,char* User,char* Pwd,char* TOPIC);
void     send_MQTT(char* topic, char* message);


#if USART3_DMA==1  
volatile u8   USART3_DMA_Tx_flag;              //DMA忙碌标志,1	
volatile u8   USART3_DMA_Rx_flag;              //DMA接收完成标志,0
void     DMA_USART3_Init(void); 
void     DMA_USART3_Tx_Data(u16 size);


#endif

#endif
