#include "sys.h"
#include "header.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
#include "SEGGER_SYSVIEW.h"

#define QUERY_TASK_PRIO 2
#define QUERY_STK_SIZE 256
TaskHandle_t QueryTask_Handler;
void query_task(void *pvParameters);



//任务优先级
#define START_TASK_PRIO			1
//任务堆栈大小	
#define START_STK_SIZE 			256  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define LED_TASK_PRIO			2
//任务堆栈大小	
#define LED_STK_SIZE 			256  
//任务句柄
TaskHandle_t LEDTask_Handler;
//任务函数
void LED_task(void *pvParameters);

//任务优先级
#define TEMP_TASK_PRIO			8
//任务堆栈大小	
#define TEMP_STK_SIZE 			256  
//任务句柄
TaskHandle_t TEMPTask_Handler;
//任务函数
void temp_task(void *pvParameters);

//任务优先级
#define INTERRUPT_TASK_PRIO		1
//任务堆栈大小	
#define INTERRUPT_STK_SIZE 		256  
//任务句柄
TaskHandle_t INTERRUPTTask_Handler;
//任务函数
void interrupt_task(void *p_arg);


//任务优先级
#define EVENT_TASK_PRIO 3
//任务堆栈大小
#define EVENT_STK_SIZE 256
//任务句柄
TaskHandle_t EVENTTASK_Handler;
//任务函数
void event_task(void *pvParameters);

//任务优先级
#define EVENTSETBIT_TASK_PRIO 4
//任务堆栈大小
#define EVENTSETBIT_STK_SIZE 256
//任务句柄
TaskHandle_t EVENTSETBITTASK_Handler;
//任务函数
void eventsetbit_task(void *pvParameters);

#define TIMERCONTROL_TASK_PRIO 8
#define TIMERCONTROL_STK_SIZE 256
TaskHandle_t TIMERCONTROLTASK_Handler;
void timercontrol_task(void *pvParameters);


TimerHandle_t AutoReloadTimer_Handle;
void AutoTimerCallback(TimerHandle_t xTimer);

EventGroupHandle_t EventGroupHandle;

void Board_Init(void);

GlobalSensor gSensor;
u8 chk;
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
	SEGGER_SYSVIEW_Conf();
	delay_init();	    				//延时函数初始化	 
	uart_init(115200);					//初始化串口
	Board_Init();
	Infra_Init();
	Ultrasonic_Init();
	Init_DS1302();
	UAVCAN_NODE_ID_SELF_ID = 8;
	CAN_Configuration();
	LED_Init();		  					//初始化LED
	chk = DS18B20_Init();
	printf("Welcome to the Sensorboard. DS18B20 = %d\r\n", chk);
//	TIM3_Int_Init(10000-1,7200-1);		//初始化定时器3，定时器周期1S
//	TIM5_Int_Init(10000-1,7200-1);		//初始化定时器5，定时器周期1S
	
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )EVENT_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )INTERRUPT_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    EventGroupHandle = xEventGroupCreate();
    //创建事件组处理任务
    xTaskCreate((TaskFunction_t )event_task,  			//任务函数
                (const char*    )"event_task", 			//任务名称
                (uint16_t       )INTERRUPT_STK_SIZE,		//任务堆栈大小
                (void*          )NULL,						//传递给任务函数的参数
                (UBaseType_t    )INTERRUPT_TASK_PRIO,		//任务优先级
                (TaskHandle_t*  )&EVENTTASK_Handler); 	//任务句柄
								
    xTaskCreate((TaskFunction_t )temp_task,  			//任务函数
                (const char*    )"temp_task", 			//任务名称
                (uint16_t       )TEMP_STK_SIZE,		//任务堆栈大小
                (void*          )NULL,						//传递给任务函数的参数
                (UBaseType_t    )TEMP_TASK_PRIO,		//任务优先级
                (TaskHandle_t*  )&TEMPTask_Handler); 	//任务句柄


		AutoReloadTimer_Handle = xTimerCreate(  (const char *) "AutoReloadTimer",
											  (TickType_t) 1000,
											  (UBaseType_t) pdTRUE,
											  (void *) 1,
											  (TimerCallbackFunction_t) AutoTimerCallback );
		if (AutoReloadTimer_Handle == NULL)
			printf("Error: fail to create the auto-reload timer\r\n");
		else
		{
			if(xTimerStart(AutoReloadTimer_Handle,0) != pdPASS)
				printf("Error:fail to start the timer\r\n");
				
		}
    //创建LED任务
    xTaskCreate((TaskFunction_t )LED_task,  			//任务函数
                (const char*    )"LED_task", 			//任务名称
                (uint16_t       )LED_STK_SIZE,		//任务堆栈大小
                (void*          )NULL,						//传递给任务函数的参数
                (UBaseType_t    )LED_TASK_PRIO,		//任务优先级
                (TaskHandle_t*  )&LEDTask_Handler); 	//任务句柄	
    //创建中断测试任务
    xTaskCreate((TaskFunction_t )interrupt_task,  			//任务函数
                (const char*    )"interrupt_task", 			//任务名称
                (uint16_t       )INTERRUPT_STK_SIZE,		//任务堆栈大小
                (void*          )NULL,						//传递给任务函数的参数
                (UBaseType_t    )INTERRUPT_TASK_PRIO,		//任务优先级
                (TaskHandle_t*  )&INTERRUPTTask_Handler); 	//任务句柄
    //创建QUERY任务
    xTaskCreate((TaskFunction_t )query_task,     
                (const char*    )"query_task",   
                (uint16_t       )QUERY_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )QUERY_TASK_PRIO,
                (TaskHandle_t*  )&QueryTask_Handler); 

	vTaskDelete(StartTask_Handler); //删除开始任务
	
  taskEXIT_CRITICAL();            //退出临界区
}


void temp_task(void *pvParameters)
{
	static short tempera =0;
	while(1)
	{
		tempera = DS18B20_Get_Temp();
		printf("Update: current temperature is %.1f\r\n", (float)tempera/10);
		Readburst(gSensor.Temperaturer.m_Time);
		printf("Time:20%d年%d月%d日%d时%d分%d秒\r\n",	gSensor.Temperaturer.m_Time[6],
																									gSensor.Temperaturer.m_Time[4],
																									gSensor.Temperaturer.m_Time[3],
																									gSensor.Temperaturer.m_Time[2],
																									gSensor.Temperaturer.m_Time[1],
																									gSensor.Temperaturer.m_Time[0]);
		vTaskDelay(10000);
	}

}

//中断测试任务函数 
void interrupt_task(void *pvParameters)
{
	static u32 total_num=0;
    while(1)
    {
		total_num+=1;
		if(total_num==5) 
		{
			printf("关闭中断.............\r\n");
			total_num = 0;
//			portDISABLE_INTERRUPTS();				//关闭中断
			delay_xms(50000);						//延时5s
			printf("打开中断.............\r\n");	//打开中断
//			portENABLE_INTERRUPTS();
		}
        vTaskDelay(1000);
    }
} 

void LED_task(void *pvParameters)
{
	while(1)
	{
		LED0 = ~LED0;
		printf("Warning:LED0 is toggled\r\n");
		vTaskDelay(1000);
	}

}


void Board_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*Initialize the Sonar Ports*/	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = TRG1|TRG2|TRG6| WATCHDOG;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = TRG3 | TRG4 | TRG5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	TRG_1 =0;
	TRG_2 =0;
	TRG_3 =0;
	TRG_4 =0;
	TRG_5 =0;
	TRG_6 =0;
}

void AutoTimerCallback(TimerHandle_t xTimer)
{
	static u16 timnum =0;
	timnum++;
	printf("Warning: Autoreload timer has worked for %d times\r\n", timnum);
	WHDG_IO = ~WHDG_IO;
//	TRG_1 =1;
//	TRG_2 =1;
//	TRG_3 =1;
//	TRG_4 =1;
//	TRG_5 =1;
//	TRG_6 =1;
//	delay_ms(2);
//	TRG_1 =0;
//	TRG_2 =0;
//	TRG_3 =0;
//	TRG_4 =0;
//	TRG_5 =0;
//	TRG_6 =0;
}

void event_task(void *pvParameters)
{
	while(1);
}

void query_task(void *pvParameters)
{
	u32 TotalRunTime;
	UBaseType_t ArraySize, x;
	TaskStatus_t *StatusArray;
	ArraySize = uxTaskGetNumberOfTasks();
	StatusArray = pvPortMalloc(ArraySize * sizeof(TaskStatus_t));
	if(StatusArray != NULL)
	{
		ArraySize = uxTaskGetSystemState( (TaskStatus_t *) StatusArray, 
											(UBaseType_t) ArraySize, 
											(uint32_t *) &TotalRunTime );
		printf("TaskName\t\tPriority\t\tTaskNumber\t\t\r\n");
		for (x=0; x<ArraySize;x++)
			{
				printf("%s\t\t%d\t\t\t%d\t\t\t\r\n", StatusArray[x].pcTaskName,
													(int)StatusArray[x].uxCurrentPriority,
													(int)StatusArray[x].xTaskNumber);
			}
	}
	vPortFree(StatusArray);
	printf("/**************************end***************************/\r\n");
	while(1)
		vTaskDelay(1000);

}
