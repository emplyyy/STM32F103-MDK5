#include "sys.h"
#include "delay.h"
#include "usart1.h"	
#include "usart2.h"
#include "usart3.h"
#include "timer2_cap.h"
#include "UltrasonicWave.h"
 
//串口几用printf在sys.c中设置，默认串口1可用printf 
int main()
{	

  unsigned char count = 0;
	delay_init(); 
	USART1_Init(115200);
//	USART2_Init(115200);
	USART3_Init(115200);

//	SoftTimer_Init();
//	Multi_Button_Init();
//	Key_Scan_Init();
	TIM2_Cap_Init(0xffff,72-1); //以1Mhz的频率计数
  UltrasonicWave_Configuration();
//	Get_IP();
//	connected_IP("192.168.1.1","1883","","admin","admin","/nbiot/test1");
//  send_MQTT("/nbiot/test1","nb test!!!");
	while(1)
	{
//		USART1_printf("11111\r\n");
//		USART3_printf("33333\r\n");

		switch(count)			
		  {case 0:UltrasonicWave_StartMeasure(); 
              count=1;
				      break;
		 	case 1:UltrasonicWave_StartMeasure2(); 
             count=2;
				     break;	
			case 2:UltrasonicWave_StartMeasure3(); 
             count=3;
				     break;		
			case 3:UltrasonicWave_StartMeasure4(); 
             count=0;
				     break;
      }
    
		printf("1:%d|2:%d|3:%d|4:%d\r\n",tempup1,tempup2,tempup3,tempup4); 
    delay_ms(1000);
    
	}
}


