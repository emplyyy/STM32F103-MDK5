#include "public.h"
u8  eu8_Flag = 0;								/*ȫ�ֱ�������һ�����жϱ�־λ*/
u8  eu8_what = 0;        			  /*ȫ�ֱ�������һ���յ���һλ����*/

#define usart2_baund  57600			/*����2�����ʣ�����ָ��ģ�鲨���ʸ���*/
SysPara AS608Para;							/*ָ��ģ��AS608����*/
u16 ValidN;											/*ģ������Чָ�Ƹ���*/

int main()
{
	u8 ensure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init();								/*��ʱ��ʼ��*/
	USART_INIT();								/*����һ��ʼ��*/
	printf("��ʼ����\r\n");
	usart2_init(usart2_baund);	/*��ʼ������2,������ָ��ģ��ͨѶ*/
	PS_StaGPIO_Init();					/*��ʼ��FR��״̬����*/
	printf("��ָ��ģ������\r\n");
	while(PS_HandShake(&AS608Addr))			/*��AS608ģ������*/
	{
		delay_ms(400);
		printf("δ��⵽ģ��\r\n");
		delay_ms(800);
		printf("������������ģ��\r\n"); 
	}
	printf("���ӳɹ�\r\n");
//	printf("������:%d   ��ַ:%x\r\n",usart2_baund,AS608Addr);		/*��ӡ��Ϣ*/
	ensure=PS_ValidTempleteNum(&ValidN);										/*����ָ�Ƹ���*/
	if(ensure!=0x00)
		ShowErrMessage(ensure);								/*��ʾȷ���������Ϣ*/
	ensure=PS_ReadSysPara(&AS608Para);  		/*������ */
	if(ensure==0x00)
	{
		printf("������:%d     �Աȵȼ�: %d",AS608Para.PS_max-ValidN,AS608Para.PS_level);
	}
	else
		ShowErrMessage(ensure);	
  while(1)
	{
//			usart1_execute();			 /*����һ���յ����������ݺ�ִ���Ӻ���*/
		
			if(PS_Sta)	 //���PS_Sta״̬���������ָ����
			{
				press_FR();//ˢָ��			
			}		
	}
}

