#include "usart1_view.h"
#include "USART_Config.h"
extern void Add_FR(u16 FR_ID);						/*¼�ĸ�ID��ָ��*/
extern void Del_FR(u16 FR_ID);						/*ɾ���ĸ�ID��ָ��*/
void usart1_execute(void)  /*����һ���յ����ݺ�ִ���Ӻ���*/
{
		extern u8 eu8_Flag;
	  extern u8 eu8_what;
		if(eu8_Flag)
		{
			printf("\r\n����ʧ��\r\n");
			eu8_Flag=0;
			switch(eu8_what)
			{
				case '1': Add_FR(1);			/*¼�ĸ�ID��ָ��*/
					break;
				case '2': Del_FR(1);			/*ɾ���ĸ�ID��ָ��*/
					break;			
				default: printf("\r\n����ʧ��\r\n");
				break;
			}		
		}
}

