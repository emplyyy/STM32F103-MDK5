#include "AS608_use.h"
#include "usart3.h"

extern SysPara AS608Para;							/*ָ��ģ��AS608����*/
extern u16 ValidN;											/*ģ������Чָ�Ƹ���*/

//��ʾȷ���������Ϣ
void ShowErrMessage(u8 ensure)
{
	printf("\r\n������Ϣ��%s \r\n",(u8*)EnsureMessage(ensure));
}

//ˢָ��
void press_FR(void)
{
	SearchResult seach;
	u8 ensure;
	ensure=PS_GetImage();
	if(ensure==0x00)//��ȡͼ��ɹ� 
	{	

		ensure=PS_GenChar(CharBuffer1);
		if(ensure==0x00) //���������ɹ�
		{		
			ensure=PS_HighSpeedSearch(CharBuffer1,0,AS608Para.PS_max,&seach);
			if(ensure==0x00)//�����ɹ�
			{				
				printf("ˢָ�Ƴɹ�\r\n");
				parseGpsBuffer();	
				printf("ID:%d �򿨳ɹ�",seach.pageID);
				printGpsBuffer();
			}
			else 
				ShowErrMessage(ensure);					
	  }
		else
			ShowErrMessage(ensure);

	}
		
}

//¼ָ��
void Add_FR(u16 FR_ID)
{
	u8 i,ensure ,processnum=0;
	u16 ID;
	while(1)
	{
		switch (processnum)
		{
			case 0:
				i++;
				printf("�밴ָ��\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(CharBuffer1);//��������
					if(ensure==0x00)
					{
						printf("ָ������\r\n");
						i=0;
						processnum=1;//�����ڶ���						
					}else ShowErrMessage(ensure);				
				}else ShowErrMessage(ensure);						
				break;
			
			case 1:
				i++;
				printf("�밴�ٰ�һ��ָ��\r\n");
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(CharBuffer2);//��������
					if(ensure==0x00)
					{
						printf("ָ������\r\n");
						i=0;
						processnum=2;//����������
					}else ShowErrMessage(ensure);	
				}else ShowErrMessage(ensure);		
				break;

			case 2:
				printf("�Ա�����ָ��\r\n");
				ensure=PS_Match();
				if(ensure==0x00) 
				{
					printf("�Աȳɹ�,����ָ��һ��\r\n");
					processnum=3;//�������Ĳ�
				}
				else 
				{
					printf("�Ա�ʧ�ܣ�������¼��ָ��\r\n");
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//���ص�һ��		
				}
				delay_ms(1200);
				break;

			case 3:
				printf("����ָ��ģ��\r\n");
				ensure=PS_RegModel();
				if(ensure==0x00) 
				{
					printf("����ָ��ģ��ɹ�\r\n");
					processnum=4;//�������岽
				}else {processnum=0;ShowErrMessage(ensure);}
				delay_ms(1200);
				break;
				
			case 4:	
				printf("����ָ��ID\r\n");
				printf("0=< ID <=299\r\n");
				do
					ID=FR_ID;
				while(!(ID<AS608Para.PS_max));//����ID����С��ָ�����������ֵ
				ensure=PS_StoreChar(CharBuffer2,ID);//����ģ��
				if(ensure==0x00) 
				{					
					printf("¼��ָ�Ƴɹ�\r\n");
					PS_ValidTempleteNum(&ValidN);//����ָ�Ƹ���
					return ;
				}else {processnum=0;ShowErrMessage(ensure);}					
				break;				
		}
		delay_ms(1000);
		if(i==5)//����5��û�а���ָ���˳�
		{
			printf("����5��û�а���ָ���˳�\r\n");
			break;	
		}				
	}
}

//ɾ��ָ��
void Del_FR(u16 FR_ID)	/*����ɾ��ָ��ID*/
{
	u8  ensure;
	printf("ɾ��ָ��\r\n");
	ensure=PS_DeletChar(FR_ID,1);//ɾ������ָ��
	if(ensure==0)
	{
		printf("ɾ��ָ�Ƴɹ�\r\n");		
	}
  else
		ShowErrMessage(ensure);	
	delay_ms(1200);
	PS_ValidTempleteNum(&ValidN);//����ָ�Ƹ���

}

