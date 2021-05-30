#include "USART2_DMA.h"

//USART2_TX   GPIOA.2
//USART2_RX	  GPIOA.3

u8   USART2_TX_BUF[USART2_MAX_TX_LEN]; 		//���ͻ���,���USART2_MAX_TX_LEN-1�ֽ�
u8   USART2_RX_BUF[USART2_MAX_RX_LEN]; 		//���ջ���,���USART2_MAX_RX_LEN-1�ֽ�
volatile u16 USART2_RX_STA=0;               //bit15:������ɱ�־   bit14~0:���յ�����Ч�ֽ���Ŀ  

//��ֲ�޸���start
//ȫ���滻.h  .c �Ĵ���USART2Ϊ��ֲ�Ĵ��ڣ����޸��������
#define USARTx                              USART2

#define USART2_TX_RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOA
#define USART2_TX_GPIOx                     GPIOA
#define USART2_TX_GPIO_Pin_x                GPIO_Pin_2

#define USART2_RX_RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOA
#define USART2_RX_GPIOx                     GPIOA
#define USART2_RX_GPIO_Pin_x                GPIO_Pin_3
 
#if USART2_DMA==1
#define RCC_AHBPeriph_DMAx                  RCC_AHBPeriph_DMA1
 
#define USART2_TX_DMAx_Channelx_IRQHandler  DMA1_Channel7_IRQHandler
#define USART2_TX_DMAx_Channely             DMA1_Channel7
#define USART2_TX_DMAx_Channely_IRQn        DMA1_Channel7_IRQn
#define USART2_TX_DMAx_FLAG_TCy             DMA1_FLAG_TC7

#define USART2_RX_DMAx_Channelx_IRQHandler  DMA1_Channel6_IRQHandler
#define USART2_RX_DMAx_Channely             DMA1_Channel6
#define USART2_RX_DMAx_Channely_IRQn        DMA1_Channel6_IRQn
#define USART2_RX_DMAx_FLAG_TCy             DMA1_FLAG_TC6
#endif
//��ֲ�޸���end

void USART2_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
    
#if USART2_DMA==1 
	while(USART2_DMA_Tx_flag == 1);//��ֹDMA���������ã���Ϸ���
#endif
    
	//ʱ������
	RCC_APB2PeriphClockCmd(USART2_TX_RCC_APB2Periph_GPIOx, ENABLE);
    RCC_APB2PeriphClockCmd(USART2_RX_RCC_APB2Periph_GPIOx, ENABLE);
    
    //�õ�ַ��ֹ��ֲȫ���滻���Ǹ�ʱ�Ӻ���
	if(USARTx==(USART_TypeDef *)(APB2PERIPH_BASE + 0x3800)) 
		RCC_APB2PeriphClockCmd(((uint32_t)0x00004000), ENABLE); //ʹ�ܴ���1ʱ��
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4400)) 
		RCC_APB1PeriphClockCmd(((uint32_t)0x00020000), ENABLE); //ʹ�ܴ���2ʱ��
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4800))          
		RCC_APB1PeriphClockCmd(((uint32_t)0x00040000), ENABLE); //ʹ�ܴ���3ʱ��
    
	//TX 
	GPIO_InitStructure.GPIO_Pin   = USART2_TX_GPIO_Pin_x; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;	 
	GPIO_Init(USART2_TX_GPIOx, &GPIO_InitStructure);

	//RX 
	GPIO_InitStructure.GPIO_Pin   = USART2_RX_GPIO_Pin_x; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(USART2_RX_GPIOx, &GPIO_InitStructure);

	//��ʼ������
	USART_InitStructure.USART_BaudRate                   = bound;                         //���ڲ�����
	USART_InitStructure.USART_WordLength                 = USART_WordLength_8b;           //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits                   = USART_StopBits_1;              //һ��ֹͣλ
	USART_InitStructure.USART_Parity                     = USART_Parity_No;               //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl        = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode                       = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);                                             //��ʼ������
	
	#if USART2_RS485==1
	My_GPIO_Init(USART2_RS485_TX_EN_GPIOx,USART2_RS485_TX_EN_Pin,GPIO_TW_OUT,GPIO_P_DOWN,GPIO_100MHz);//������� ���� 100m
	USART2_RS485_TX_EN=0;
	#endif
	
	//NVIC����
	NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);	 
  
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);                                        //�������ڿ���IDEL�ж�
	USART_Cmd(USART2, ENABLE);                                                            //ʹ�ܴ��� 
    
#if USART2_DMA==0
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                                        //�������ڽ����ж�
#else
    DMA_USART2_Init();                                                                    //���� DMA ����
#endif
 
	
}
 
#if USART2_DMA==1 
void DMA_USART2_Init(void)
{
    DMA_InitTypeDef DMA_USART2_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMAx , ENABLE );
 
	/*---- DMA_UART_Tx_DMA_Channel DMA Config ---*/ 
    DMA_Cmd(USART2_TX_DMAx_Channely, DISABLE);                                             //��DMAͨ��
    DMA_DeInit(USART2_TX_DMAx_Channely);                                                   //�ָ�ȱʡֵ
    DMA_USART2_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART2->DR));           //ע������ط�ȡ2�ε�ַ  ���ô��ڷ������ݼĴ���
    DMA_USART2_InitStructure.DMA_MemoryBaseAddr          = (u32)USART2_TX_BUF;             //���÷��ͻ������׵�ַ
    DMA_USART2_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralDST;          //��������λĿ�꣬�ڴ滺���� -> ����Ĵ���
    DMA_USART2_InitStructure.DMA_BufferSize              = USART2_MAX_TX_LEN;              //��Ҫ���͵��ֽ�����������ʵ��������Ϊ0����Ϊ��ʵ��Ҫ���͵�ʱ�򣬻��������ô�ֵ
    DMA_USART2_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART2_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //�ڴ滺������ַ���ӵ���
    DMA_USART2_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //�������ݿ��8λ��1���ֽ�
    DMA_USART2_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART2_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //���δ���ģʽ����ѭ��
    DMA_USART2_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //���ȼ�����
    DMA_USART2_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(USART2_TX_DMAx_Channely, &DMA_USART2_InitStructure);                          //д������
    
    DMA_ClearFlag(USART2_TX_DMAx_FLAG_TCy);                                                //���DMA���б�־    
    DMA_Cmd(USART2_TX_DMAx_Channely, DISABLE);                                             //�ر�DMA
    DMA_ITConfig(USART2_TX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //��������DMAͨ���ж�
   
    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_TX_DMAx_Channely_IRQn;   // ����DMAͨ�����ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                              // ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);                                         //��������DMA����
    
	/*--- DMA_UART_Rx_DMA_Channel DMA Config ---*/
    DMA_Cmd(USART2_RX_DMAx_Channely, DISABLE);                                             //��DMAͨ��
    DMA_DeInit(USART2_RX_DMAx_Channely);                                                   //�ָ�ȱʡֵ
    DMA_USART2_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART2->DR));           //ע������ط�ȡ��ַ���ô��ڽ������ݼĴ���
	DMA_USART2_InitStructure.DMA_MemoryBaseAddr          = (u32)USART2_RX_BUF;             //���ý��ջ������׵�ַ
    DMA_USART2_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralSRC;          //DMA_DIR_PeripheralSRC:�������  DMA_DIR_PeripheralDST:���ڴ��
    DMA_USART2_InitStructure.DMA_BufferSize              = USART2_MAX_RX_LEN;              //��Ҫ�����ܽ��յ����ֽ���
    DMA_USART2_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART2_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //�ڴ滺������ַ���ӵ���
    DMA_USART2_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //�������ݿ��8λ��1���ֽ�
    DMA_USART2_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART2_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //���δ���ģʽ����ѭ��
    DMA_USART2_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //���ȼ�����
    DMA_USART2_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //�Ƿ����ڴ浽�ڴ洫��(�رձ�ʾֻ���¼������Ŵ���һ�����ݣ�������ʾһֱ����)DMA_M2M_Disable:�Ǵ洢�����洢��ģʽ(�ر��ڴ浽�ڴ�ģʽ)  DMA_M2M_Enable:�����洢�����洢��ģʽ(�����ڴ浽�ڴ�ģʽ)
    DMA_Init(USART2_RX_DMAx_Channely, &DMA_USART2_InitStructure);                          //д������
		
    DMA_ClearFlag(USART2_RX_DMAx_FLAG_TCy);                                                //���DMA���б�־
    DMA_Cmd(USART2_RX_DMAx_Channely, ENABLE);                                              //��������DMAͨ�����ȴ���������
	DMA_ITConfig(USART2_RX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //��������DMAͨ���ж�
    
    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_RX_DMAx_Channely_IRQn;   // ����DMAͨ�����ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                              // ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);                                         //��������DMA����
}
 
//DMA ����Ӧ��Դ��
void DMA_USART2_Tx_Data(u16 size)                                                //DMA����
{
	#if USART2_RS485==1
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);                //�ȴ���һ֡�������
	USART2_RS485_TX_EN=1;
	#endif
	
	USART2_DMA_Tx_flag = 1;
	USART2_TX_DMAx_Channely->CNDTR =size;                                        //����Ҫ���͵��ֽ���Ŀ
    DMA_Cmd(USART2_TX_DMAx_Channely, ENABLE);                                    //��ʼDMA����
}

void USART2_TX_DMAx_Channelx_IRQHandler(void)
{
    if(DMA_GetITStatus(USART2_TX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART2_TX_DMAx_FLAG_TCy);                                  //�����־
		DMA_Cmd(USART2_TX_DMAx_Channely, DISABLE);                               //�ر�DMAͨ��
		USART2_DMA_Tx_flag = 0;                                                  //�������
 
        #if USART2_RS485==1
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC)  == RESET);            //�ȴ���һ֡��ȫ���ͳ�ȥ
        USART2_RS485_TX_EN=0;
        #endif
    }
}

//DMA ����Ӧ��Դ��
void DMA_USART2_Rx_Data(void)
{
 
}

void USART2_RX_DMAx_Channelx_IRQHandler(void)                //DMA���գ�����USART2_MAX_RX_LEN������
{
    if(DMA_GetITStatus(USART2_RX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART2_RX_DMAx_FLAG_TCy);              //�����־
		DMA_Cmd(USART2_RX_DMAx_Channely, DISABLE);           //�ر�DMA ����ֹ����
    }
}
#endif

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2 , USART_IT_IDLE) != RESET)                                   //�����ж�
    {
		USART_ReceiveData(USART2);                                                           //��������жϱ�־
#if USART2_DMA==1
		DMA_Cmd(USART2_RX_DMAx_Channely, DISABLE);                                           //�ر�DMA ����ֹ����  		
 
		USART2_RX_STA = USART2_MAX_RX_LEN - DMA_GetCurrDataCounter(USART2_RX_DMAx_Channely); //��ý��յ����ֽ���
		USART2_RX_DMAx_Channely->CNDTR = USART2_MAX_RX_LEN;                                  //���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
        
		DMA_Cmd(USART2_RX_DMAx_Channely, ENABLE);                                            //DMA �������ȴ����ݡ�
#endif
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]='\0';		                                     //���\0,��ֹ�ַ���������������\0һֱ������
		USART2_RX_STA|=0x8000;	                                                             //��ǽ��������
        //��Ӵ�����,�����������ѯ����
			
    }
	if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) == SET)                                    // ��� ORE ��־,��ֹ�������ж����������ڽ����ж�ǰ��
	{
		USART_ClearFlag(USART2,USART_FLAG_ORE);
		USART_ReceiveData(USART2);
	}
#if USART2_DMA==0
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)                                    //�����ж� 
	{
		u8 res = USART_ReceiveData(USART2);	                                                 //��ȡ���յ�������	
		if((USART2_RX_STA&0X7FFF)<USART2_MAX_RX_LEN-1)                                       //�������鳤�ȵ�����,�����жϺ�������ǰ�������ݻ��������
		{
			USART2_RX_BUF[USART2_RX_STA&0X7FFF]=res;
			USART2_RX_STA++;
		}
	}
#endif
}
 
void USART2_printf (char *fmt, ...)
{
#if USART2_DMA==1 
	u16 i = 0;

	while(USART2_DMA_Tx_flag==1);
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART2_TX_BUF, USART2_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);

	DMA_USART2_Tx_Data(strlen((char *)USART2_TX_BUF));
#else
	u16 i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART2_TX_BUF, USART2_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);
	
	#if USART2_RS485==1
	USART2_RS485_TX_EN=1;
	#endif
	
	while ((i < USART2_MAX_TX_LEN) && USART2_TX_BUF[i])
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
		USART_SendData(USART2, (u8) USART2_TX_BUF[i++]);	
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC)  == RESET);
	}
	
	#if USART2_RS485==1
	USART2_RS485_TX_EN=0;
	#endif
#endif
}
 
void USART2_Send_Array (u8 *array,u16 num)
{
#if USART2_DMA==1
	while(USART2_DMA_Tx_flag==1);
	memcpy(USART2_TX_BUF,array,num);
	DMA_USART2_Tx_Data(num);
#else
	u16 i = 0;
	
	#if USART2_RS485==1
	USART2_RS485_TX_EN=1;
	#endif
	
	while (i < num)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
		USART_SendData(USART2, (u8) array[i++]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC)  == RESET);
	}
	
	#if USART2_RS485==1
	USART2_RS485_TX_EN=0;
	#endif
#endif
}
 