#include "USART_Config.h"
void USART_Config(void)
{
  USART_InitTypeDef  USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate=9600;  //9600������
  USART_InitStructure.USART_WordLength=USART_WordLength_8b;//8Ϊ����λ	
  USART_InitStructure.USART_StopBits=USART_StopBits_1; //1λֹͣλ		
  USART_InitStructure.USART_Parity=USART_Parity_No;		//��У��λ
  USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //������Ӳ����
  USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;	 //ȫ˫��
	USART_Init(USART1,&USART_InitStructure);   
	
	USART_Cmd(USART1, ENABLE);    //ʹ�ܻ���ʧ�� USART ����
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//ʹ�ܻ���ʧ��ָ���� USART �ж�
	USART_ClearFlag(USART1,USART_FLAG_TC);//��� USARTx �Ĵ������־λ
	
	
}
void USART_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;   //����	
  SystemInit();  //ϵͳ����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//���ܸ��� IO ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //GPIOA ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //USART1ʱ��
	
	/************�������������********************/
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;          //�����������  ����
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;              //GPIO PA0 ��  
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/************�������������********************/
	
	/************��������***********************/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10; //����
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING; //��������
  GPIO_Init(GPIOA,&GPIO_InitStructure);   
	/************��������***********************/
	
}	
void USART_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
	  
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	  //��ռ���ȼ� 1 λ,�����ȼ� 3 λ
	  NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;;   //ʹ�ܻ���ʧ��ָ���� IRQ ͨ������ǲ�һ��  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�Ϊ0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //��Ӧ���ȼ�Ϊ0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 		  //ʹ��
		NVIC_Init(&NVIC_InitStructure); 
	
}	

void USART_INIT(void)
{
		USART_GPIO_Config();
		USART_NVIC_Config();
		USART_Config();
}

int fputc(int ch, FILE *f)
{
		USART_SendData(USART1, (unsigned char) ch);
		while (!(USART1->SR & USART_FLAG_TXE));
		return (ch);
}
int GetKey (void) 
{
		while (!(USART1->SR & USART_FLAG_RXNE));
		return ((int)(USART1->DR & 0x1FF));
}
	
void USART1_IRQHandler()
{
	extern u8 eu8_Flag;
	extern u8 eu8_what;
  USART_ClearFlag(USART1,USART_FLAG_TC);
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=Bit_RESET)//���ָ����USART�жϷ������
	{
		eu8_Flag=1;
		eu8_what = USART_ReceiveData(USART1);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
	}
}  

		
