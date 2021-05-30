#include "USART1_DMA.h"

//USART1_TX   GPIOA.9
//USART1_RX	  GPIOA.10

u8   USART1_TX_BUF[USART1_MAX_TX_LEN]; 		//���ͻ���,���USART1_MAX_TX_LEN-1�ֽ�
u8   USART1_RX_BUF[USART1_MAX_RX_LEN]; 		//���ջ���,���USART1_MAX_RX_LEN-1�ֽ�
volatile u16 USART1_RX_STA=0;               //bit15:������ɱ�־   bit14~0:���յ�����Ч�ֽ���Ŀ    

//��ֲ�޸���start
//ȫ���滻.h  .c �Ĵ���USART1Ϊ��ֲ�Ĵ��ڣ����޸��������
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
//��ֲ�޸���end

void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
    
#if USART1_DMA==1 
	while(USART1_DMA_Tx_flag == 1);//��ֹDMA���������ã���Ϸ���
#endif
    
	//ʱ������
	RCC_APB2PeriphClockCmd(USART1_TX_RCC_APB2Periph_GPIOx, ENABLE);
    RCC_APB2PeriphClockCmd(USART1_RX_RCC_APB2Periph_GPIOx, ENABLE);
    
    //�õ�ַ��ֹ��ֲȫ���滻���Ǹ�ʱ�Ӻ���
	if(USARTx==(USART_TypeDef *)(APB2PERIPH_BASE + 0x3800)) 
		RCC_APB2PeriphClockCmd(((uint32_t)0x00004000), ENABLE); //ʹ�ܴ���1ʱ��
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4400)) 
		RCC_APB1PeriphClockCmd(((uint32_t)0x00020000), ENABLE); //ʹ�ܴ���2ʱ��
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4800))          
		RCC_APB1PeriphClockCmd(((uint32_t)0x00040000), ENABLE); //ʹ�ܴ���3ʱ��
    
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

	//��ʼ������
	USART_InitStructure.USART_BaudRate                   = bound;                         //���ڲ�����
	USART_InitStructure.USART_WordLength                 = USART_WordLength_8b;           //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits                   = USART_StopBits_1;              //һ��ֹͣλ
	USART_InitStructure.USART_Parity                     = USART_Parity_No;               //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl        = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode                       = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);                                             //��ʼ������
	
	#if USART1_RS485==1
	My_GPIO_Init(USART1_RS485_TX_EN_GPIOx,USART1_RS485_TX_EN_Pin,GPIO_TW_OUT,GPIO_P_DOWN,GPIO_100MHz);//������� ���� 100m
	USART1_RS485_TX_EN=0;
	#endif
	
	//NVIC����
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);	 
  
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                                        //�������ڿ���IDEL�ж�
	USART_Cmd(USART1, ENABLE);                                                            //ʹ�ܴ��� 
    
#if USART1_DMA==0
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                        //�������ڽ����ж�
#else
    DMA_USART1_Init();                                                                    //���� DMA ����
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
    DMA_Cmd(USART1_TX_DMAx_Channely, DISABLE);                                             //��DMAͨ��
    DMA_DeInit(USART1_TX_DMAx_Channely);                                                   //�ָ�ȱʡֵ
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART1->DR));           //ע������ط�ȡ2�ε�ַ  ���ô��ڷ������ݼĴ���
    DMA_USART1_InitStructure.DMA_MemoryBaseAddr          = (u32)USART1_TX_BUF;             //���÷��ͻ������׵�ַ
    DMA_USART1_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralDST;          //��������λĿ�꣬�ڴ滺���� -> ����Ĵ���
    DMA_USART1_InitStructure.DMA_BufferSize              = USART1_MAX_TX_LEN;              //��Ҫ���͵��ֽ�����������ʵ��������Ϊ0����Ϊ��ʵ��Ҫ���͵�ʱ�򣬻��������ô�ֵ
    DMA_USART1_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART1_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //�ڴ滺������ַ���ӵ���
    DMA_USART1_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //�������ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //���δ���ģʽ����ѭ��
    DMA_USART1_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //���ȼ�����
    DMA_USART1_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(USART1_TX_DMAx_Channely, &DMA_USART1_InitStructure);                          //д������
    
    DMA_ClearFlag(USART1_TX_DMAx_FLAG_TCy);                                                //���DMA���б�־    
    DMA_Cmd(USART1_TX_DMAx_Channely, DISABLE);                                             //�ر�DMA
    DMA_ITConfig(USART1_TX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //��������DMAͨ���ж�
   
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_TX_DMAx_Channely_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);                                         //��������DMA����
    
	/*--- DMA_UART_Rx_DMA_Channel DMA Config ---*/
    DMA_Cmd(USART1_RX_DMAx_Channely, DISABLE);                                             //��DMAͨ��
    DMA_DeInit(USART1_RX_DMAx_Channely);                                                   //�ָ�ȱʡֵ
    DMA_USART1_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART1->DR));           //ע������ط�ȡ��ַ���ô��ڽ������ݼĴ���
	DMA_USART1_InitStructure.DMA_MemoryBaseAddr          = (u32)USART1_RX_BUF;             //���ý��ջ������׵�ַ
    DMA_USART1_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralSRC;          //DMA_DIR_PeripheralSRC:�������  DMA_DIR_PeripheralDST:���ڴ��
    DMA_USART1_InitStructure.DMA_BufferSize              = USART1_MAX_RX_LEN;              //��Ҫ�����ܽ��յ����ֽ���
    DMA_USART1_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART1_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //�ڴ滺������ַ���ӵ���
    DMA_USART1_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //�������ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART1_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //���δ���ģʽ����ѭ��
    DMA_USART1_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //���ȼ�����
    DMA_USART1_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //�Ƿ����ڴ浽�ڴ洫��(�رձ�ʾֻ���¼������Ŵ���һ�����ݣ�������ʾһֱ����)DMA_M2M_Disable:�Ǵ洢�����洢��ģʽ(�ر��ڴ浽�ڴ�ģʽ)  DMA_M2M_Enable:�����洢�����洢��ģʽ(�����ڴ浽�ڴ�ģʽ)
    DMA_Init(USART1_RX_DMAx_Channely, &DMA_USART1_InitStructure);                          //д������
		
    DMA_ClearFlag(USART1_RX_DMAx_FLAG_TCy);                                                //���DMA���б�־
    DMA_Cmd(USART1_RX_DMAx_Channely, ENABLE);                                              //��������DMAͨ�����ȴ���������
	DMA_ITConfig(USART1_RX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //��������DMAͨ���ж�
    
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_RX_DMAx_Channely_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);                                         //��������DMA����
}
 
//DMA ����Ӧ��Դ��
void DMA_USART1_Tx_Data(u16 size)                                                //DMA����
{
	#if USART1_RS485==1
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);                //�ȴ���һ֡�������
	USART1_RS485_TX_EN=1;
	#endif
	
	USART1_DMA_Tx_flag = 1;
	USART1_TX_DMAx_Channely->CNDTR =size;                                        //����Ҫ���͵��ֽ���Ŀ
    DMA_Cmd(USART1_TX_DMAx_Channely, ENABLE);                                    //��ʼDMA����
}

void USART1_TX_DMAx_Channely_IRQHandler(void)
{
    if(DMA_GetITStatus(USART1_TX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART1_TX_DMAx_FLAG_TCy);                                  //�����־
		DMA_Cmd(USART1_TX_DMAx_Channely, DISABLE);                               //�ر�DMAͨ��
		USART1_DMA_Tx_flag = 0;                                                  //�������
 
        #if USART1_RS485==1
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC)  == RESET);            //�ȴ���һ֡��ȫ���ͳ�ȥ
        USART1_RS485_TX_EN=0;
        #endif
    }
}

//DMA ����Ӧ��Դ��(��û�ã��õĿ����ж�)
void DMA_USART1_Rx_Data(void)
{
 
}

void USART1_RX_DMAx_Channely_IRQHandler(void)                //DMA���գ�����USART1_MAX_RX_LEN������
{
    if(DMA_GetITStatus(USART1_RX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART1_RX_DMAx_FLAG_TCy);              //�����־
		DMA_Cmd(USART1_RX_DMAx_Channely, DISABLE);           //�ر�DMA ����ֹ����
    }
}
#endif

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1 , USART_IT_IDLE) != RESET)                                   //�����ж�
    {
		USART_ReceiveData(USART1);                                                           //��������жϱ�־
#if USART1_DMA==1
		DMA_Cmd(USART1_RX_DMAx_Channely, DISABLE);                                           //�ر�DMA ����ֹ����  		
 
		USART1_RX_STA = USART1_MAX_RX_LEN - DMA_GetCurrDataCounter(USART1_RX_DMAx_Channely); //��ý��յ����ֽ���
		DMA_SetCurrDataCounter(USART1_RX_DMAx_Channely,USART1_MAX_RX_LEN);                   //���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
        
		DMA_Cmd(USART1_RX_DMAx_Channely, ENABLE);                                            //DMA �������ȴ����ݡ�
#endif
		USART1_RX_BUF[USART1_RX_STA&0X7FFF]='\0';		                                     //���\0,��ֹ�ַ���������������\0һֱ������
		USART1_RX_STA|=0x8000;	                                                             //��ǽ��������
        //��Ӵ�����,�����������ѯ����
			
    }
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET)                                    // ��� ORE ��־,��ֹ�������ж����������ڽ����ж�ǰ��
	{
		USART_ClearFlag(USART1,USART_FLAG_ORE);
		USART_ReceiveData(USART1);
	}
#if USART1_DMA==0
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)                                    //�����ж� 
	{
		u8 res = USART_ReceiveData(USART1);	                                                 //��ȡ���յ�������	
		if((USART1_RX_STA&0X7FFF)<USART1_MAX_RX_LEN-1)                                       //�������鳤�ȵ�����,�����жϺ�������ǰ�������ݻ��������
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
