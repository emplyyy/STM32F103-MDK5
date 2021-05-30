#include "USART3_DMA.h"

//USART3_TX   GPIOB.10
//USART3_RX	  GPIOB.11

u8   USART3_TX_BUF[USART3_MAX_TX_LEN]; 		//���ͻ���,���USART3_MAX_TX_LEN-1�ֽ�
u8   USART3_RX_BUF[USART3_MAX_RX_LEN]; 		//���ջ���,���USART3_MAX_RX_LEN-1�ֽ�
volatile u16 USART3_RX_STA=0;               //bit15:������ɱ�־   bit14~0:���յ�����Ч�ֽ���Ŀ  

//��ֲ�޸���start
//ȫ���滻.h  .c �Ĵ���USART3Ϊ��ֲ�Ĵ��ڣ����޸��������
#define USARTx                              USART3

#define USART3_TX_RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOB
#define USART3_TX_GPIOx                     GPIOB
#define USART3_TX_GPIO_Pin_x                GPIO_Pin_10

#define USART3_RX_RCC_APB2Periph_GPIOx      RCC_APB2Periph_GPIOB
#define USART3_RX_GPIOx                     GPIOB
#define USART3_RX_GPIO_Pin_x                GPIO_Pin_11
 
#if USART3_DMA==1
#define RCC_AHBPeriph_DMAx                  RCC_AHBPeriph_DMA1
 
#define USART3_TX_DMAx_Channelx_IRQHandler  DMA1_Channel2_IRQHandler
#define USART3_TX_DMAx_Channely             DMA1_Channel2
#define USART3_TX_DMAx_Channely_IRQn        DMA1_Channel2_IRQn
#define USART3_TX_DMAx_FLAG_TCy             DMA1_FLAG_TC2

#define USART3_RX_DMAx_Channelx_IRQHandler  DMA1_Channel3_IRQHandler
#define USART3_RX_DMAx_Channely             DMA1_Channel3
#define USART3_RX_DMAx_Channely_IRQn        DMA1_Channel3_IRQn
#define USART3_RX_DMAx_FLAG_TCy             DMA1_FLAG_TC3
#endif
//��ֲ�޸���end

void USART3_Init(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
    
#if USART3_DMA==1 
	while(USART3_DMA_Tx_flag == 1);//��ֹDMA���������ã���Ϸ���
#endif
    
	//ʱ������
	RCC_APB2PeriphClockCmd(USART3_TX_RCC_APB2Periph_GPIOx, ENABLE);
    RCC_APB2PeriphClockCmd(USART3_RX_RCC_APB2Periph_GPIOx, ENABLE);
    
    //�õ�ַ��ֹ��ֲȫ���滻���Ǹ�ʱ�Ӻ���
	if(USARTx==(USART_TypeDef *)(APB2PERIPH_BASE + 0x3800)) 
		RCC_APB2PeriphClockCmd(((uint32_t)0x00004000), ENABLE); //ʹ�ܴ���1ʱ��
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4400)) 
		RCC_APB1PeriphClockCmd(((uint32_t)0x00020000), ENABLE); //ʹ�ܴ���2ʱ��
	if(USARTx==(USART_TypeDef *)(APB1PERIPH_BASE + 0x4800))          
		RCC_APB1PeriphClockCmd(((uint32_t)0x00040000), ENABLE); //ʹ�ܴ���3ʱ��
    
	//TX 
	GPIO_InitStructure.GPIO_Pin   = USART3_TX_GPIO_Pin_x; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;	 
	GPIO_Init(USART3_TX_GPIOx, &GPIO_InitStructure);

	//RX 
	GPIO_InitStructure.GPIO_Pin   = USART3_RX_GPIO_Pin_x; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(USART3_RX_GPIOx, &GPIO_InitStructure);

	//��ʼ������
	USART_InitStructure.USART_BaudRate                   = bound;                         //���ڲ�����
	USART_InitStructure.USART_WordLength                 = USART_WordLength_8b;           //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits                   = USART_StopBits_1;              //һ��ֹͣλ
	USART_InitStructure.USART_Parity                     = USART_Parity_No;               //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl        = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode                       = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure);                                             //��ʼ������
	
	#if USART3_RS485==1
	My_GPIO_Init(USART3_RS485_TX_EN_GPIOx,USART3_RS485_TX_EN_Pin,GPIO_TW_OUT,GPIO_P_DOWN,GPIO_100MHz);//������� ���� 100m
	USART3_RS485_TX_EN=0;
	#endif
	
	//NVIC����
	NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);	 
  
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);                                        //�������ڿ���IDEL�ж�
	USART_Cmd(USART3, ENABLE);                                                            //ʹ�ܴ��� 
    
#if USART3_DMA==0
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                        //�������ڽ����ж�
#else
    DMA_USART3_Init();                                                                    //���� DMA ����
#endif
 
	
}
 
#if USART3_DMA==1 
void DMA_USART3_Init(void)
{
    DMA_InitTypeDef DMA_USART3_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMAx , ENABLE );
 
	/*---- DMA_UART_Tx_DMA_Channel DMA Config ---*/ 
    DMA_Cmd(USART3_TX_DMAx_Channely, DISABLE);                                             //��DMAͨ��
    DMA_DeInit(USART3_TX_DMAx_Channely);                                                   //�ָ�ȱʡֵ
    DMA_USART3_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART3->DR));           //ע������ط�ȡ2�ε�ַ  ���ô��ڷ������ݼĴ���
    DMA_USART3_InitStructure.DMA_MemoryBaseAddr          = (u32)USART3_TX_BUF;             //���÷��ͻ������׵�ַ
    DMA_USART3_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralDST;          //��������λĿ�꣬�ڴ滺���� -> ����Ĵ���
    DMA_USART3_InitStructure.DMA_BufferSize              = USART3_MAX_TX_LEN;              //��Ҫ���͵��ֽ�����������ʵ��������Ϊ0����Ϊ��ʵ��Ҫ���͵�ʱ�򣬻��������ô�ֵ
    DMA_USART3_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART3_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //�ڴ滺������ַ���ӵ���
    DMA_USART3_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //�������ݿ��8λ��1���ֽ�
    DMA_USART3_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART3_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //���δ���ģʽ����ѭ��
    DMA_USART3_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //���ȼ�����
    DMA_USART3_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(USART3_TX_DMAx_Channely, &DMA_USART3_InitStructure);                          //д������
    
    DMA_ClearFlag(USART3_TX_DMAx_FLAG_TCy);                                                //���DMA���б�־    
    DMA_Cmd(USART3_TX_DMAx_Channely, DISABLE);                                             //�ر�DMA
    DMA_ITConfig(USART3_TX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //��������DMAͨ���ж�
   
    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_TX_DMAx_Channely_IRQn;   // ����DMAͨ�����ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                              // ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);                                         //��������DMA����
    
	/*--- DMA_UART_Rx_DMA_Channel DMA Config ---*/
    DMA_Cmd(USART3_RX_DMAx_Channely, DISABLE);                                             //��DMAͨ��
    DMA_DeInit(USART3_RX_DMAx_Channely);                                                   //�ָ�ȱʡֵ
    DMA_USART3_InitStructure.DMA_PeripheralBaseAddr      = (u32)(&(USART3->DR));           //ע������ط�ȡ��ַ���ô��ڽ������ݼĴ���
	DMA_USART3_InitStructure.DMA_MemoryBaseAddr          = (u32)USART3_RX_BUF;             //���ý��ջ������׵�ַ
    DMA_USART3_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralSRC;          //DMA_DIR_PeripheralSRC:�������  DMA_DIR_PeripheralDST:���ڴ��
    DMA_USART3_InitStructure.DMA_BufferSize              = USART3_MAX_RX_LEN;              //��Ҫ�����ܽ��յ����ֽ���
    DMA_USART3_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;      //�����ַ�������ӵ�����������������DMA�Զ�ʵ�ֵ�
    DMA_USART3_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;           //�ڴ滺������ַ���ӵ���
    DMA_USART3_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;    //�������ݿ��8λ��1���ֽ�
    DMA_USART3_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;        //�ڴ����ݿ��8λ��1���ֽ�
    DMA_USART3_InitStructure.DMA_Mode                    = DMA_Mode_Normal;                //���δ���ģʽ����ѭ��
    DMA_USART3_InitStructure.DMA_Priority                = DMA_Priority_VeryHigh;          //���ȼ�����
    DMA_USART3_InitStructure.DMA_M2M                     = DMA_M2M_Disable;                //�Ƿ����ڴ浽�ڴ洫��(�رձ�ʾֻ���¼������Ŵ���һ�����ݣ�������ʾһֱ����)DMA_M2M_Disable:�Ǵ洢�����洢��ģʽ(�ر��ڴ浽�ڴ�ģʽ)  DMA_M2M_Enable:�����洢�����洢��ģʽ(�����ڴ浽�ڴ�ģʽ)
    DMA_Init(USART3_RX_DMAx_Channely, &DMA_USART3_InitStructure);                          //д������
		
    DMA_ClearFlag(USART3_RX_DMAx_FLAG_TCy);                                                //���DMA���б�־
    DMA_Cmd(USART3_RX_DMAx_Channely, ENABLE);                                              //��������DMAͨ�����ȴ���������
	DMA_ITConfig(USART3_RX_DMAx_Channely, DMA_IT_TC, ENABLE);                              //��������DMAͨ���ж�
    
    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_RX_DMAx_Channely_IRQn;   // ����DMAͨ�����ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                              // ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);                                         //��������DMA����
}
 
//DMA ����Ӧ��Դ��
void DMA_USART3_Tx_Data(u16 size)                                                //DMA����
{
	#if USART3_RS485==1
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);                //�ȴ���һ֡�������
	USART3_RS485_TX_EN=1;
	#endif
	
	USART3_DMA_Tx_flag = 1;
	USART3_TX_DMAx_Channely->CNDTR =size;                                        //����Ҫ���͵��ֽ���Ŀ
    DMA_Cmd(USART3_TX_DMAx_Channely, ENABLE);                                    //��ʼDMA����
}

void USART3_TX_DMAx_Channelx_IRQHandler(void)
{
    if(DMA_GetITStatus(USART3_TX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART3_TX_DMAx_FLAG_TCy);                                  //�����־
		DMA_Cmd(USART3_TX_DMAx_Channely, DISABLE);                               //�ر�DMAͨ��
		USART3_DMA_Tx_flag = 0;                                                  //�������
 
        #if USART3_RS485==1
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC)  == RESET);            //�ȴ���һ֡��ȫ���ͳ�ȥ
        USART3_RS485_TX_EN=0;
        #endif
    }
}

//DMA ����Ӧ��Դ��
void DMA_USART3_Rx_Data(void)
{
 
}

void USART3_RX_DMAx_Channelx_IRQHandler(void)                //DMA���գ�����USART3_MAX_RX_LEN������
{
    if(DMA_GetITStatus(USART3_RX_DMAx_FLAG_TCy))
    {
		DMA_ClearFlag(USART3_RX_DMAx_FLAG_TCy);              //�����־
		DMA_Cmd(USART3_RX_DMAx_Channely, DISABLE);           //�ر�DMA ����ֹ����
    }
}
#endif

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3 , USART_IT_IDLE) != RESET)                                   //�����ж�
    {
		USART_ReceiveData(USART3);                                                           //��������жϱ�־
#if USART3_DMA==1
		DMA_Cmd(USART3_RX_DMAx_Channely, DISABLE);                                           //�ر�DMA ����ֹ����  		
 
		USART3_RX_STA = USART3_MAX_RX_LEN - DMA_GetCurrDataCounter(USART3_RX_DMAx_Channely); //��ý��յ����ֽ���
		USART3_RX_DMAx_Channely->CNDTR = USART3_MAX_RX_LEN;                                  //���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
        
		DMA_Cmd(USART3_RX_DMAx_Channely, ENABLE);                                            //DMA �������ȴ����ݡ�
#endif
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]='\0';		                                     //���\0,��ֹ�ַ���������������\0һֱ������
		USART3_RX_STA|=0x8000;	                                                             //��ǽ��������
        //��Ӵ�����,�����������ѯ����
			
    }
	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE) == SET)                                    // ��� ORE ��־,��ֹ�������ж����������ڽ����ж�ǰ��
	{
		USART_ClearFlag(USART3,USART_FLAG_ORE);
		USART_ReceiveData(USART3);
	}
#if USART3_DMA==0
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)                                    //�����ж� 
	{
		u8 res = USART_ReceiveData(USART3);	                                                 //��ȡ���յ�������	
		if((USART3_RX_STA&0X7FFF)<USART3_MAX_RX_LEN-1)                                       //�������鳤�ȵ�����,�����жϺ�������ǰ�������ݻ��������
		{
			USART3_RX_BUF[USART3_RX_STA&0X7FFF]=res;
			USART3_RX_STA++;
		}
	}
#endif
}
 
void USART3_printf (char *fmt, ...)
{
#if USART3_DMA==1 
	u16 i = 0;

	while(USART3_DMA_Tx_flag==1);
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART3_TX_BUF, USART3_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);

	DMA_USART3_Tx_Data(strlen((char *)USART3_TX_BUF));
#else
	u16 i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, fmt); 
	vsnprintf((char *)USART3_TX_BUF, USART3_MAX_TX_LEN+1, fmt, arg_ptr);
	va_end(arg_ptr);
	
	#if USART3_RS485==1
	USART3_RS485_TX_EN=1;
	#endif
	
	while ((i < USART3_MAX_TX_LEN) && USART3_TX_BUF[i])
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
		USART_SendData(USART3, (u8) USART3_TX_BUF[i++]);	
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC)  == RESET);
	}
	
	#if USART3_RS485==1
	USART3_RS485_TX_EN=0;
	#endif
#endif
}
 
void USART3_Send_Array (u8 *array,u16 num)
{
#if USART3_DMA==1
	while(USART3_DMA_Tx_flag==1);
	memcpy(USART3_TX_BUF,array,num);
	DMA_USART3_Tx_Data(num);
#else
	u16 i = 0;
	
	#if USART3_RS485==1
	USART3_RS485_TX_EN=1;
	#endif
	
	while (i < num)
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
		USART_SendData(USART3, (u8) array[i++]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC)  == RESET);
	}
	
	#if USART3_RS485==1
	USART3_RS485_TX_EN=0;
	#endif
#endif
}
 

int NBiot_SendCmd(char* cmd, char* reply, int wait)
{
	memset(USART3_RX_BUF,0,USART3_MAX_RX_LEN);//����ջ�����
  USART3_printf("%s\r\n", cmd);
  delay_ms(100);   /* ��ʱ100ms*/		 		
  delay_ms(wait); 
  if (strcmp(USART3_RX_BUF, "") == 0) //����ֵΪ��
  {
		
		USART3_printf("NONE!!\r\n");
    return 0;
  }
	if (strstr((char*)USART3_RX_BUF, reply))   //�Ƿ��������
	{
		USART3_printf("YES!!\r\n");
		return 1;
	}
	else if (strstr((char*)USART3_RX_BUF, "ERROR"))
	{
		USART3_printf("ERROR!!\r\n");
		return 0;
	}
	else
	{
		USART3_printf("No!!\r\n");
		return 0;
	}
 
}

/*
m5311��ʼ��
*/
int NBiot_Init(void)  
{
	  
    int ret = 0;
	  delay_ms(2000);
    ret = NBiot_SendCmd("AT+CMRB", "OK", 2000); //��λ
		while (!NBiot_SendCmd("AT", "OK", 2000));
		if ( NBiot_SendCmd("AT+CMSYSCTRL=0,2", "OK", 2000))  
			USART3_printf("%s\r\n", "M5311 IS READY!"); //����ָʾ��
    return ret;
}


/*
m5311�����ʼ��
*/
int Get_IP(void)
{
	int ret = 0;
	ret = NBiot_Init();
  int try_time2 = 0;
  while (!NBiot_SendCmd("AT+CGPADDR=1", "+CGPADDR: 1", 1000)) //ѭ�����ӣ�ֱ�����ӵ�������
  {
    try_time2 += 1;
    USART3_printf("Try_Time:%d\r\n", try_time2);
    if (try_time2 > 100) {
      try_time2 = 0;
      NBiot_Init();
    }
  }
	return ret;
}

/*
m5311����mqtt������
*/
void connected_IP(char* IP, char*Port, char* ClientID, char* User, char* Pwd,char* TOPIC)
{
  int try_time = 0;
	char Connect [100];
	char SUB_TOPIC [100];
  delay_ms(3000);
	sprintf(Connect,"%s%s%s%s%s%s%s%s%s%s%s","AT+MQTTCFG=\"",IP,"\",",Port,",\"",ClientID,"\",60,\"",User,"\",\"",Pwd,"\",1");
	sprintf(SUB_TOPIC,"%s%s%s","AT+MQTTSUB=\"",TOPIC,"_back\",1");
	//printf("Connect:%s",Connect);
	//printf("SUB_TOPIC:%s",SUB_TOPIC);
  Get_IP();  //ѭ�����ӣ�ֱ�����ӵ�������
  delay_ms(1000);
//IWDG_Feed(); //ιһ�ι�
  if (NBiot_SendCmd(Connect,"OK", 2000)) {
  delay_ms(1000);
  if (NBiot_SendCmd("AT+MQTTOPEN=1,1,0,0,0,\"\",\"\"", "OK", 5000)) {
      		
      USART3_printf("%s", "connected_success��");
      while (!NBiot_SendCmd(SUB_TOPIC,"OK",2000)) {
          try_time += 1;
          USART3_printf("Try_Time:%d\r\n", try_time);
          if (try_time == 5) {
            try_time = 0;
            return;
          }
        }
       USART3_printf("%s", "SUB_TOPIC_success��");
      } else 
	    USART3_printf("%s", "connected_error��");
    }

}

/*
m5311������Ϣ
*/
void send_MQTT(char* topic, char* message)
{
	u8 error_time = 0;
  char* head = "AT+MQTTPUB=\"";
  char* modle = "\",1,1,0,0";
  char* fu_L = ",\"";
  char* fu_R = "\"";
  char output [100];
  sprintf(output, "%s%s%s%s%s%s", head, topic, modle, fu_L, message, fu_R); //ƴ���ַ���
  if (!NBiot_SendCmd(output, "+MQTTPUBACK", 3000))
  {
    error_time += 1; 
    USART3_printf("error_time:%d\r\n", error_time);
  }
  else error_time = 0 ;
  if (error_time >= 5) // �������error 5��  ����
  {
   
		error_time = 0;
  }
  memset(output,0,sizeof(output));/*�������� */
}
