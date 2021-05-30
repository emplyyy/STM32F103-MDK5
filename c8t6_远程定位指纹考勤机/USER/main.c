#include "public.h"
u8  eu8_Flag = 0;								/*全局变量串口一发生中断标志位*/
u8  eu8_what = 0;        			  /*全局变量串口一接收到的一位数据*/

#define usart2_baund  57600			/*串口2波特率，根据指纹模块波特率更改*/
SysPara AS608Para;							/*指纹模块AS608参数*/
u16 ValidN;											/*模块内有效指纹个数*/

int main()
{
	u8 ensure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init();								/*延时初始化*/
	USART_INIT();								/*串口一初始化*/
	printf("初始化中\r\n");
	usart2_init(usart2_baund);	/*初始化串口2,用于与指纹模块通讯*/
	PS_StaGPIO_Init();					/*初始化FR读状态引脚*/
	printf("与指纹模块握手\r\n");
	while(PS_HandShake(&AS608Addr))			/*与AS608模块握手*/
	{
		delay_ms(400);
		printf("未检测到模块\r\n");
		delay_ms(800);
		printf("尝试重新连接模块\r\n"); 
	}
	printf("连接成功\r\n");
//	printf("波特率:%d   地址:%x\r\n",usart2_baund,AS608Addr);		/*打印信息*/
	ensure=PS_ValidTempleteNum(&ValidN);										/*读库指纹个数*/
	if(ensure!=0x00)
		ShowErrMessage(ensure);								/*显示确认码错误信息*/
	ensure=PS_ReadSysPara(&AS608Para);  		/*读参数 */
	if(ensure==0x00)
	{
		printf("库容量:%d     对比等级: %d",AS608Para.PS_max-ValidN,AS608Para.PS_level);
	}
	else
		ShowErrMessage(ensure);	
  while(1)
	{
//			usart1_execute();			 /*串口一接收到串口屏数据后执行子函数*/
		
			if(PS_Sta)	 //检测PS_Sta状态，如果有手指按下
			{
				press_FR();//刷指纹			
			}		
	}
}

