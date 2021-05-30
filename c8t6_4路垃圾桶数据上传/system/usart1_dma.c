#include "USART1_DMA.h"

//USART1_TX   GPIOA.9
//USART1_RX	  GPIOA.10

u8   USART1_TX_BUF[USART1_MAX_TX_LEN]; 		//发送缓冲,最大USART1_MAX_TX_LEN-1字节
u8   USART1_RX_BUF[USART1_MAX_RX_LEN]; 		//接收缓冲,最大USART1_MAX_RX_LEN-1字节
volatile u16 USART1_RX_STA=0;               //bit15:接收完成标志   bit14~0:接收到的有效字节数目    

//移植修改区start
//全局替换.h  .c 的串口USART1为移植的串口，并修改下面代码
#define USARTx                              USART1

#define USART1_TX_RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOA
#define USART1_TX_GPIOx                     GPIOA
#define USART1_TX_GPIO_Pin_x                GPIO_Pin_9

#define USART1_RX_RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOA
#define USART1_RX_GPIOx                     GPIOA
#define USART1_RX_GPIO_Pin_x                GPIO_Pin_10
 
#if USART1_DMA==1
#define RCC_AHBPeriph_DMAx                  RCC_AHBPeriph_DMA1
 
#define USART1_TX_DMAx_Channely_IRQHandler  DMA1_Channel4_IRQHandler
#define USART1_TX_DMAx_Channely             DMA1_Channel4
#define USART1_TX_DMAx_Channely_IRQn        DMA1_Channel4_IRQn
#define USART1_TX_DMAx_FLAG_TCy             DMA1_FLAG_TC4

#define USART1_RX_DMAx_Channely_IRQHandler  DMA1_Channel5_IRQHandler
#define USART1_RX_DMAx_Channely             DMA1_Channel5
#define USART1_RX_DMAx_Channely_IRQn        DMA1_Channel5_IRQn
#define USART1_RX_DMAx_FLAG_TCy             DMA1_FLAG_TC5
#endif
//移植修改区end

void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
    
#if USART1_DMA==1 
	while(USART1_DMA_Tx_flag == 1);//防止DMA发送中设置，打断发送
#endif
    
	//时钟设置
	RCC_APB2PeriphClockCmd(USART1_TX_RCC_APB2Periph_GPIOx, ENABLE);
    RCC_APB2PeriphClockCmd(USART1_RX_RCC_APB2Periph_GPIOx, ENABLE);
    
    //用地址防止移植全局替换忘记改时钟函数
	if(USARTx==(USART_TypeDef *)(APB2PERIPH_BASE + 0x3800)) 
		RCC_APB2PeriphClockCmd(((uint32_t)0x00004000), ENABLE); //使能串口1时钟
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4400)) 
		RCC_APB1PeriphClockCmd(((uint32_t)0x00020000), ENABLE); //使能串口2时钟
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4800))          
		RCC_APB1PeriphClockCmd(((uint32_t)0x00040000), ENABLE); //使能串口3时钟
    
	//TX 
	GPIO_InitStructure.GPIO_Pin   = USART1_TX_GPIO_Pin_x; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;	 
	GPIO_Init(USART1_TX_GPIOx, &GPIO_InitStructure);

	//RX 
	GPIO_InitStructure.GPIO_Pin   = USART1_RX_GPIO_Pin_x; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(USART1_RX_GPIOx, &GPIO_InitStructure);

	//初始化设置
	USART_InitStructure.USART_BaudRate                   = bound;                         //串口波特率
	USART_InitStructure.USART_WordLength                 = USART_WordLength_8b;           //字长为8位数据格式
	USART_InitStructure.USART_StopBits                   = USART_StopBits_1;              //一个停止位
	USART_InitStructure.USART_Parity                     = USART_Parity_No;               //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl        = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode                       = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_Init(USART1, &USART_InitStructure);                                             //初始化串口
	
	#if USART1_RS485==1
	My_GPIO_Init(USART1_RS485_TX_EN_GPIOx,USART1_RS485_TX_EN_Pin,GPIO_TW_OUT,GPIO_P_DOWN,GPIO_100MHz);//推挽输出 下拉 100m
	USART1_RS485_TX_EN=0;
	#endif
	
	//NVIC配置
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);	 
  
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                                        //开启串口空闲IDEL中断
	USART_Cmd(USART1, ENABLE);                                                            //使能串口 
    
#if USART1_DMA==0
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                        //开启串口接受中断
#else
    DMA_USART1_Init();                                                                    //串口 DMA 配置
#endif
 
	USART1_printf("USART1 OK...\r\n");
}
 
#if USART1_DMA==1 
void DMA_USART1_Init(void)
{
    DMA_InitTypeDef DMA_USART1_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMAx , ENABLE );
 
	/*---- DMA_UART_Tx_DMA_Channel DMA Config ---*/ 
    DMA_Cmd(USART1_TX_DMAx_Channely, DISABLE);                                             //关DMA通道
    DMA_DeInit(USART1_TX_DMAx_Channely);                                                   //恢复缺省值
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART1->DR));           //注意这个地方取2次地址  设置串口发送数据寄存器
    DMA_USART1_InitStructure.DMA_MemoryBaseAddr          = (u32)USART1_TX_BUF;             //设置发送缓冲区首地址
    DMA_USART1_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralDST;          //设置外设位目标，内存缓冲区 -> 外设寄存器
    DMA_USART1_InitStructure.DMA_BufferSize              = USART1_MAX_TX_LEN;              //需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值
    DMA_USART1_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_USART1_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //内存缓冲区地址增加调整
    DMA_USART1_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //外设数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //内存数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //单次传输模式，不循环
    DMA_USART1_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //优先级设置
    DMA_USART1_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //关闭内存到内存的DMA模式
    DMA_Init(USART1_TX_DMAx_Channely, &DMA_USART1_InitStructure);                          //写入配置
    
    DMA_ClearFlag(USART1_TX_DMAx_FLAG_TCy);                                                //清除DMA所有标志    
    DMA_Cmd(USART1_TX_DMAx_Channely, DISABLE);                                             //关闭DMA
    DMA_ITConfig(USART1_TX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //开启发送DMA通道中断
   
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_TX_DMAx_Channely_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);                                         //开启串口DMA发送
    
	/*--- DMA_UART_Rx_DMA_Channel DMA Config ---*/
    DMA_Cmd(USART1_RX_DMAx_Channely, DISABLE);                                             //关DMA通道
    DMA_DeInit(USART1_RX_DMAx_Channely);                                                   //恢复缺省值
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART1->DR));           //注意这个地方取地址设置串口接收数据寄存器
	DMA_USART1_InitStructure.DMA_MemoryBaseAddr          = (u32)USART1_RX_BUF;             //设置接收缓冲区首地址
    DMA_USART1_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralSRC;          //DMA_DIR_PeripheralSRC:从外设读  DMA_DIR_PeripheralDST:从内存读
    DMA_USART1_InitStructure.DMA_BufferSize              = USART1_MAX_RX_LEN;              //需要最大可能接收到的字节数
    DMA_USART1_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_USART1_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //内存缓冲区地址增加调整
    DMA_USART1_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //外设数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //内存数据宽度8位，1个字节
    DMA_USART1_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //单次传输模式，不循环
    DMA_USART1_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //优先级设置
    DMA_USART1_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //是否开启内存到内存传输(关闭表示只有事件产生才传输一次数据，开启表示一直传输)DMA_M2M_Disable:非存储器到存储器模式(关闭内存到内存模式)  DMA_M2M_Enable:启动存储器到存储器模式(开启内存到内存模式)
    DMA_Init(USART1_RX_DMAx_Channely, &DMA_USART1_InitStructure);                          //写入配置
		
    DMA_ClearFlag(USART1_RX_DMAx_FLAG_TCy);                                                //清除DMA所有标志
    DMA_Cmd(USART1_RX_DMAx_Channely, ENABLE);                                              //开启接收DMA通道，等待接收数据
	DMA_ITConfig(USART1_RX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //开启接收DMA通道中断
    
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_RX_DMAx_Channely_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);                                         //开启串口DMA接收
}
 
//DMA 发送应用源码
void DMA_USART1_Tx_Data(u16 size)                                                //DMA发送
{
	#if USART1_RS485==1
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);                //等待上一帧发送完毕
	USART1_RS485_TX_EN=1;
	#endif
	
	USART1_DMA_Tx_flag = 1;
	USART1_TX_DMAx_Channely->CNDTR =size;                                        //设置要发送的字节数目
    DMA_Cmd(USART1_TX_DMAx_Channely, ENABLE);                                    //开始DMA发送
}

void USART1_TX_DMAx_Channely_IRQHandler(void)
{
    if(DMA_GetITStatus(USART1_TX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART1_TX_DMAx_FLAG_TCy);                                  //清除标志
		DMA_Cmd(USART1_TX_DMAx_Channely, DISABLE);                               //关闭DMA通道
		USART1_DMA_Tx_flag = 0;                                                  //发送完成
 
        #if USART1_RS485==1
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC)  == RESET);            //等待这一帧完全发送除去
        USART1_RS485_TX_EN=0;
        #endif
    }
}

//DMA 接收应用源码(都没用，用的空闲中断)
void DMA_USART1_Rx_Data(void)
{
 
}

void USART1_RX_DMAx_Channely_IRQHandler(void)                //DMA接收，超过USART1_MAX_RX_LEN的舍弃
{
    if(DMA_GetITStatus(USART1_RX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART1_RX_DMAx_FLAG_TCy);              //清除标志
		DMA_Cmd(USART1_RX_DMAx_Channely, DISABLE);           //关闭DMA ，防止干扰
    }
}
#endif

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1 , USART_IT_IDLE) != RESET)                                   //空闲中断
    {
		USART_ReceiveData(USART1);                                                           //清除空闲中断标志
#if USART1_DMA==1
		DMA_Cmd(USART1_RX_DMAx_Channely, DISABLE);                                           //关闭DMA ，防止干扰  		
 
		USART1_RX_STA = USART1_MAX_RX_LEN - DMA_GetCurrDataCounter(USART1_RX_DMAx_Channely); //获得接收到的字节数
		DMA_SetCurrDataCounter(USART1_RX_DMAx_Channely,USART1_MAX_RX_LEN);                   //重新赋值计数值，必须大于等于最大可能接收到的数据帧数目
        
		DMA_Cmd(USART1_RX_DMAx_Channely, ENABLE);                                            //DMA 开启，等待数据。
#endif
		USART1_RX_BUF[USART1_RX_STA&0X7FFF]='\0';		                                     //添加\0,防止字符串处理函数遇不见\0一直不结束
		USART1_RX_STA|=0x8000;	                                                             //标记接收完成了
        //添加处理函数,最好主函数查询处理
			
    }
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET)                                    // 检查 ORE 标志,防止开关总中断死机，放在接收中断前面
	{
		USART_ClearFlag(USART1,USART_FLAG_ORE);
		USART_ReceiveData(USART1);
	}
#if USART1_DMA==0
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)                                    //接收中断 
	{
		u8 res = USART_ReceiveData(USART1);	                                                 //读取接收到的数据	
		if((USART1_RX_STA&0X7FFF)<USART1_MAX_RX_LEN-1)                                       //超过数组长度的舍弃,空闲中断后处理数据前来的数据会继续接上
		{
			USART1_RX_BUF[USART1_RX_STA&0X7FFF]=res;
			USART1_RX_STA++;
		}
	}
#endif
}
 
void USART1_printf (char *fmt, ...)
{
#if USART1_DMA==1 
	u16 i = 0;

	while(USART1_DMA_Tx_flag==1);
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART1_TX_BUF, USART1_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);

	DMA_USART1_Tx_Data(strlen((char *)USART1_TX_BUF));
#else
	u16 i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART1_TX_BUF, USART1_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);
	
	#if USART1_RS485==1
	USART1_RS485_TX_EN=1;
	#endif
	
	while ((i < USART1_MAX_TX_LEN) && USART1_TX_BUF[i])
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
		USART_SendData(USART1, (u8) USART1_TX_BUF[i++]);	
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC)  == RESET);
	}
	
	#if USART1_RS485==1
	USART1_RS485_TX_EN=0;
	#endif
#endif
}
 
void USART1_Send_Array (u8 *array,u16 num)
{
#if USART1_DMA==1
	while(USART1_DMA_Tx_flag==1);
	memcpy(USART1_TX_BUF,array,num);
	DMA_USART1_Tx_Data(num);
#else
	u16 i = 0;
	
	#if USART1_RS485==1
	USART1_RS485_TX_EN=1;
	#endif
	
	while (i < num)
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
		USART_SendData(USART1, (u8) array[i++]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC)  == RESET);
	}
	
	#if USART1_RS485==1
	USART1_RS485_TX_EN=0;
	#endif
#endif
}
 
void uartScreen(u8 a, u8 b, u8 c, u8 d) {
	
	USART1_printf("DC24(30,5,'%d',18);\r\n",a);
	delay_ms(1000);
	USART1_printf("DC24(112,5,'%d',18);\r\n",b);
	delay_ms(1000);
	USART1_printf("DC24(112,72,'%d',18);\r\n",c);
	delay_ms(1000);
	USART1_printf("DC24(30,72,'%d',18);\r\n",d);
	delay_ms(1000);
	

}
