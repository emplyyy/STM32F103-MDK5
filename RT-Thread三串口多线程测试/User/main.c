/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   RT-Thread 3.0 + STM32 线程管理
  *********************************************************************
  * @attention
  *
  * 实验平台:野火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 

/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/ 
#include "board.h"
#include "stdio.h"
#include "rtthread.h"
#include <string.h>
#include <ctype.h>


/*
*************************************************************************
*                               变量
*************************************************************************
*/
/* 定义线程控制块 */
static rt_thread_t usart_thread = RT_NULL; //led线程
static rt_thread_t usart2_thread = RT_NULL;//串口2线程
static rt_thread_t usart3_thread = RT_NULL;//串口3线程

/* 定义信号量控制块 */
rt_sem_t test_sem2 = RT_NULL;
rt_sem_t test_sem3 = RT_NULL;
rt_sem_t JSON_sem = RT_NULL;
rt_sem_t MQTT_sem = RT_NULL;
/************************* 全局变量声明 ****************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */

/* 相关串口缓存数 */
extern char Usart2_Rx_Buf[USART2_RBUFF_SIZE];
extern char Usart3_Rx_Buf[USART3_RBUFF_SIZE];

/*
*************************************************************************
*                             函数声明
*************************************************************************
*/

/*线程声明*/
static void usart_thread_entry(void* parameter);
static void usart2_thread_entry(void* parameter);
static void usart3_thread_entry(void* parameter);
/*
*************************************************************************
*                             main 函数
*************************************************************************
*/

int main(void)
{	
    /* 
	 * 开发板硬件初始化，RTT系统初始化已经在main函数之前完成，
	 * 即在component.c文件中的rtthread_startup()函数中完成了。
	 * 所以在main函数中，只需要创建线程和启动线程即可。
	 */


//*******************************************************************************//	
		usart_thread =                          /* 线程控制块指针 */
    rt_thread_create( "test_sem1",              /* 线程名字 */
                      usart_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (usart_thread != RT_NULL)
        rt_thread_startup(usart_thread);
    else
        return -1;
//*******************************************************************************//	
		
		/* 创建一个信号量 */
	test_sem2 = rt_sem_create("test_sem2",/* 消息队列名字 */
                     0,     /* 信号量初始值，默认有一个信号量 */
                     RT_IPC_FLAG_FIFO); /* 信号量模式 FIFO(0x00)*/
	MQTT_sem = rt_sem_create("MQTT_sem",/* 消息队列名字 */
                     0,     /* 信号量初始值，默认有一个信号量 */
                     RT_IPC_FLAG_FIFO); /* 信号量模式 FIFO(0x00)*/	
  if (test_sem2 != RT_NULL)
    rt_kprintf("信号量2创建成功！\n\n");
    
  usart2_thread =                          /* 线程控制块指针 */
    rt_thread_create( "usart2",              /* 线程名字 */
                      usart2_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      1024,                 /* 线程栈大小 */
                      4,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (usart2_thread != RT_NULL)
        rt_thread_startup(usart2_thread);
    else
        return -1;
//*******************************************************************************//			
			/* 创建一个信号量 */
	test_sem3 = rt_sem_create("test_sem3",/* 消息队列名字 */
                     0,     /* 信号量初始值，默认有一个信号量 */
                     RT_IPC_FLAG_FIFO); /* 信号量模式 FIFO(0x00)*/
	JSON_sem = rt_sem_create("JSON_sem",/* 消息队列名字 */
                     0,     /* 信号量初始值，默认有一个信号量 */
                     RT_IPC_FLAG_FIFO); /* 信号量模式 FIFO(0x00)*/	
  if (test_sem3 != RT_NULL)
    rt_kprintf("信号量3创建成功！n\n");
    
  usart3_thread =                          /* 线程控制块指针 */
    rt_thread_create( "usart3",              /* 线程名字 */
                      usart3_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      5,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (usart3_thread != RT_NULL)
        rt_thread_startup(usart3_thread);
    else
        return -1;	
}

/*
*************************************************************************
*                             线程定义
*************************************************************************
*/

static void usart_thread_entry(void* parameter)
{	
  
    while (1)
    {
			
				 LED1_OFF;  
				 rt_thread_delay(1000);
			   Usart_SendString(USART1,"usart1 runing\r\n");
         LED1_ON;
			   rt_thread_delay(1000);


    }
}
//*******************************************************************************//	

//*******************************************************************************//	
static void usart2_thread_entry(void* parameter)  //串口二
{
  rt_err_t uwRet = RT_EOK;	
	
    /* 任务都是一个无限循环，不能返回 */
	Usart2_SendString(USART1,"usart2 runing");
	rt_thread_delay(1000);

  while (1)
  {
	                                                          
		uwRet = rt_sem_take(test_sem2,	/* 获取串口中断的信号量 */
                        0); 	  /* 等待时间：0 */
 
		rt_thread_delay(1000);
    if(RT_EOK == uwRet)
    {
			
//			Usart2_SendString(USART2,"串口2收到数据：");
			Usart2_SendString(USART2,Usart2_Rx_Buf);
      memset(Usart2_Rx_Buf,0,USART2_RBUFF_SIZE);/* 清零 */

    }
  }
}
//*******************************************************************************//	
static void usart3_thread_entry(void* parameter)  //串口三
{
  rt_err_t uwRet = RT_EOK;	
	
    /* 任务都是一个无限循环，不能返回 */
	
	Usart3_SendString(USART1,"usart3 runing");
	rt_thread_delay(1000);
	
  while (1)
  {
		
		uwRet = rt_sem_take(test_sem3,	/* 获取串口中断的信号量 */
                        0); 	  /* 等待时间：0 */
    if(RT_EOK == uwRet)
    {
			
//			Usart3_SendString(USART3,"串口3收到数据：");
	  	Usart3_SendString(USART3,Usart3_Rx_Buf);
      memset(Usart3_Rx_Buf,0,USART3_RBUFF_SIZE);/* 清零 */

    }
  }
}
