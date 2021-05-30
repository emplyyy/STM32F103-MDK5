/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   RT-Thread 3.0 + STM32 �̹߳���
  *********************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-�Ե� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 

/*
*************************************************************************
*                             ������ͷ�ļ�
*************************************************************************
*/ 
#include "board.h"
#include "stdio.h"
#include "rtthread.h"
#include <string.h>
#include <ctype.h>


/*
*************************************************************************
*                               ����
*************************************************************************
*/
/* �����߳̿��ƿ� */
static rt_thread_t usart_thread = RT_NULL; //led�߳�
static rt_thread_t usart2_thread = RT_NULL;//����2�߳�
static rt_thread_t usart3_thread = RT_NULL;//����3�߳�

/* �����ź������ƿ� */
rt_sem_t test_sem2 = RT_NULL;
rt_sem_t test_sem3 = RT_NULL;
rt_sem_t JSON_sem = RT_NULL;
rt_sem_t MQTT_sem = RT_NULL;
/************************* ȫ�ֱ������� ****************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩȫ�ֱ�����
 */

/* ��ش��ڻ����� */
extern char Usart2_Rx_Buf[USART2_RBUFF_SIZE];
extern char Usart3_Rx_Buf[USART3_RBUFF_SIZE];

/*
*************************************************************************
*                             ��������
*************************************************************************
*/

/*�߳�����*/
static void usart_thread_entry(void* parameter);
static void usart2_thread_entry(void* parameter);
static void usart3_thread_entry(void* parameter);
/*
*************************************************************************
*                             main ����
*************************************************************************
*/

int main(void)
{	
    /* 
	 * ������Ӳ����ʼ����RTTϵͳ��ʼ���Ѿ���main����֮ǰ��ɣ�
	 * ����component.c�ļ��е�rtthread_startup()����������ˡ�
	 * ������main�����У�ֻ��Ҫ�����̺߳������̼߳��ɡ�
	 */


//*******************************************************************************//	
		usart_thread =                          /* �߳̿��ƿ�ָ�� */
    rt_thread_create( "test_sem1",              /* �߳����� */
                      usart_thread_entry,   /* �߳���ں��� */
                      RT_NULL,             /* �߳���ں������� */
                      512,                 /* �߳�ջ��С */
                      3,                   /* �̵߳����ȼ� */
                      20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
   if (usart_thread != RT_NULL)
        rt_thread_startup(usart_thread);
    else
        return -1;
//*******************************************************************************//	
		
		/* ����һ���ź��� */
	test_sem2 = rt_sem_create("test_sem2",/* ��Ϣ�������� */
                     0,     /* �ź�����ʼֵ��Ĭ����һ���ź��� */
                     RT_IPC_FLAG_FIFO); /* �ź���ģʽ FIFO(0x00)*/
	MQTT_sem = rt_sem_create("MQTT_sem",/* ��Ϣ�������� */
                     0,     /* �ź�����ʼֵ��Ĭ����һ���ź��� */
                     RT_IPC_FLAG_FIFO); /* �ź���ģʽ FIFO(0x00)*/	
  if (test_sem2 != RT_NULL)
    rt_kprintf("�ź���2�����ɹ���\n\n");
    
  usart2_thread =                          /* �߳̿��ƿ�ָ�� */
    rt_thread_create( "usart2",              /* �߳����� */
                      usart2_thread_entry,   /* �߳���ں��� */
                      RT_NULL,             /* �߳���ں������� */
                      1024,                 /* �߳�ջ��С */
                      4,                   /* �̵߳����ȼ� */
                      20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
   if (usart2_thread != RT_NULL)
        rt_thread_startup(usart2_thread);
    else
        return -1;
//*******************************************************************************//			
			/* ����һ���ź��� */
	test_sem3 = rt_sem_create("test_sem3",/* ��Ϣ�������� */
                     0,     /* �ź�����ʼֵ��Ĭ����һ���ź��� */
                     RT_IPC_FLAG_FIFO); /* �ź���ģʽ FIFO(0x00)*/
	JSON_sem = rt_sem_create("JSON_sem",/* ��Ϣ�������� */
                     0,     /* �ź�����ʼֵ��Ĭ����һ���ź��� */
                     RT_IPC_FLAG_FIFO); /* �ź���ģʽ FIFO(0x00)*/	
  if (test_sem3 != RT_NULL)
    rt_kprintf("�ź���3�����ɹ����\n\n");
    
  usart3_thread =                          /* �߳̿��ƿ�ָ�� */
    rt_thread_create( "usart3",              /* �߳����� */
                      usart3_thread_entry,   /* �߳���ں��� */
                      RT_NULL,             /* �߳���ں������� */
                      512,                 /* �߳�ջ��С */
                      5,                   /* �̵߳����ȼ� */
                      20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
   if (usart3_thread != RT_NULL)
        rt_thread_startup(usart3_thread);
    else
        return -1;	
}

/*
*************************************************************************
*                             �̶߳���
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
static void usart2_thread_entry(void* parameter)  //���ڶ�
{
  rt_err_t uwRet = RT_EOK;	
	
    /* ������һ������ѭ�������ܷ��� */
	Usart2_SendString(USART1,"usart2 runing");
	rt_thread_delay(1000);

  while (1)
  {
	                                                          
		uwRet = rt_sem_take(test_sem2,	/* ��ȡ�����жϵ��ź��� */
                        0); 	  /* �ȴ�ʱ�䣺0 */
 
		rt_thread_delay(1000);
    if(RT_EOK == uwRet)
    {
			
//			Usart2_SendString(USART2,"����2�յ����ݣ�");
			Usart2_SendString(USART2,Usart2_Rx_Buf);
      memset(Usart2_Rx_Buf,0,USART2_RBUFF_SIZE);/* ���� */

    }
  }
}
//*******************************************************************************//	
static void usart3_thread_entry(void* parameter)  //������
{
  rt_err_t uwRet = RT_EOK;	
	
    /* ������һ������ѭ�������ܷ��� */
	
	Usart3_SendString(USART1,"usart3 runing");
	rt_thread_delay(1000);
	
  while (1)
  {
		
		uwRet = rt_sem_take(test_sem3,	/* ��ȡ�����жϵ��ź��� */
                        0); 	  /* �ȴ�ʱ�䣺0 */
    if(RT_EOK == uwRet)
    {
			
//			Usart3_SendString(USART3,"����3�յ����ݣ�");
	  	Usart3_SendString(USART3,Usart3_Rx_Buf);
      memset(Usart3_Rx_Buf,0,USART3_RBUFF_SIZE);/* ���� */

    }
  }
}
