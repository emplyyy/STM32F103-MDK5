/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   重定向c库printf函数到usart端口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火STM32 F103-霸道 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "bsp_usart2.h"
#include "rtthread.h"
#include "string.h"

/* 外部定义信号量控制块 */
extern rt_sem_t test_sem2;
extern rt_sem_t MQTT_sem;

 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration2(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART2_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* 子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void USART2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// 打开串口GPIO的时钟
	DEBUG_USART2_GPIO_APBxClkCmd(DEBUG_USART2_GPIO_CLK, ENABLE);
	
	// 打开串口外设的时钟
	DEBUG_USART2_APBxClkCmd(DEBUG_USART2_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART2_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART2_TX_GPIO_PORT, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART2_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART2_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART2_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USART2, &USART_InitStructure);
	// 串口中断优先级配置
	NVIC_Configuration2();
	// 开启 串口空闲IDEL 中断
	USART_ITConfig(DEBUG_USART2, USART_IT_IDLE, ENABLE);  
  // 开启串口DMA接收
	USART_DMACmd(DEBUG_USART2, USART_DMAReq_Rx, ENABLE); 
	// 使能串口
	USART_Cmd(DEBUG_USART2, ENABLE);	    
}

char Usart2_Rx_Buf[USART2_RBUFF_SIZE];

void USART2_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;
	
		// 开启DMA时钟
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		// 设置DMA源地址：串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USART2_DR_ADDRESS;
		// 内存地址(要传输的变量的指针)
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Usart2_Rx_Buf;
		// 方向：从内存到外设	
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		// 传输大小	
		DMA_InitStructure.DMA_BufferSize = USART2_RBUFF_SIZE;
		// 外设地址不增	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		// 内存地址自增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		// 外设数据单位	
		DMA_InitStructure.DMA_PeripheralDataSize = 
	  DMA_PeripheralDataSize_Byte;
		// 内存数据单位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
		// DMA模式，一次或者循环模式
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ; //单次模式
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 //循环模式
		// 优先级：中	
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
		// 禁止内存到内存的传输
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		// 配置DMA通道		   
		DMA_Init(USART2_RX_DMA_CHANNEL, &DMA_InitStructure);		
    // 清除DMA所有标志
		DMA_ClearFlag(DMA1_FLAG_GL6);
    ///DMA_ClearFlag(DMA1_FLAG_GL5);
    DMA_ITConfig(USART2_RX_DMA_CHANNEL, DMA_IT_TE, ENABLE);
		// 使能DMA
		DMA_Cmd (USART2_RX_DMA_CHANNEL,ENABLE);
}
//终于搞定串口2的DMA接收  但是为什么串口1就不需要中断配置呢  很费解！！！！
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  // 空闲中断

    {
        Uart2_DMA_Rx_Data();
	//		if (strstr((char*)Usart2_Rx_Buf, "2:"))   //是否包含数据
		  	rt_sem_release( MQTT_sem);  //回传信号量		
        USART_ReceiveData( USART2 ); // Clear IDLE interrupt flag bit
    }
}

void Uart2_DMA_Rx_Data(void)
{
   // 关闭DMA ，防止干扰
   DMA_Cmd(USART2_RX_DMA_CHANNEL, DISABLE);      
   // 清DMA标志位
	 DMA_ClearFlag( DMA1_FLAG_GL6 ); 
   ///DMA_ClearFlag( DMA1_FLAG_GL5 );          
   //  重新赋值计数值，必须大于等于最大可能接收到的数据帧数目
   USART2_RX_DMA_CHANNEL->CNDTR = USART2_RBUFF_SIZE;    
   DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);       
   //给出二值信号量 ，发送接收到新数据标志，供前台程序查询
   rt_sem_release(test_sem2);  	
  /* 
    DMA 开启，等待数据。注意，如果中断发送数据帧的速率很快，MCU来不及处理此次接收到的数据，
    中断又发来数据的话，这里不能开启，否则数据会被覆盖。有2种方式解决：

    1. 在重新开启接收DMA通道之前，将LumMod_Rx_Buf缓冲区里面的数据复制到另外一个数组中，
    然后再开启DMA，然后马上处理复制出来的数据。

    2. 建立双缓冲，在LumMod_Uart_DMA_Rx_Data函数中，重新配置DMA_MemoryBaseAddr 的缓冲区地址，
    那么下次接收到的数据就会保存到新的缓冲区中，不至于被覆盖。
  */
}


/*****************  发送一个字节 **********************/
void Usart2_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart2_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart2_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart2_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart2_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*******************************************************************************
  发送字节
*******************************************************************************/
void uart2_send_char(u8 temp)
{
  USART_SendData(USART2, (u8)temp);
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/*******************************************************************************
  发送字符串
*******************************************************************************/
void uart2_send_buff(u8 buf[], u32 len)
{
  u32 i;
  for (i = 0; i < len; i++)
    uart2_send_char(buf[i]);
  // 可设置换行符   NB串口AT指令需要换行
  uart2_send_char('\r');
  uart2_send_char('\n');
}
/*****************  发送一个16位数 **********************/
void Usart2_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}


