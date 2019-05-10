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



//�������ȼ�
#define START_TASK_PRIO			1
//�����ջ��С	
#define START_STK_SIZE 			256  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define LED_TASK_PRIO			2
//�����ջ��С	
#define LED_STK_SIZE 			256  
//������
TaskHandle_t LEDTask_Handler;
//������
void LED_task(void *pvParameters);

//�������ȼ�
#define TEMP_TASK_PRIO			8
//�����ջ��С	
#define TEMP_STK_SIZE 			256  
//������
TaskHandle_t TEMPTask_Handler;
//������
void temp_task(void *pvParameters);

//�������ȼ�
#define INTERRUPT_TASK_PRIO		1
//�����ջ��С	
#define INTERRUPT_STK_SIZE 		256  
//������
TaskHandle_t INTERRUPTTask_Handler;
//������
void interrupt_task(void *p_arg);


//�������ȼ�
#define EVENT_TASK_PRIO 3
//�����ջ��С
#define EVENT_STK_SIZE 256
//������
TaskHandle_t EVENTTASK_Handler;
//������
void event_task(void *pvParameters);

//�������ȼ�
#define EVENTSETBIT_TASK_PRIO 4
//�����ջ��С
#define EVENTSETBIT_STK_SIZE 256
//������
TaskHandle_t EVENTSETBITTASK_Handler;
//������
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	SEGGER_SYSVIEW_Conf();
	delay_init();	    				//��ʱ������ʼ��	 
	uart_init(115200);					//��ʼ������
	Board_Init();
	Infra_Init();
	Ultrasonic_Init();
	Init_DS1302();
	UAVCAN_NODE_ID_SELF_ID = 8;
	CAN_Configuration();
	LED_Init();		  					//��ʼ��LED
	chk = DS18B20_Init();
	printf("Welcome to the Sensorboard. DS18B20 = %d\r\n", chk);
//	TIM3_Int_Init(10000-1,7200-1);		//��ʼ����ʱ��3����ʱ������1S
//	TIM5_Int_Init(10000-1,7200-1);		//��ʼ����ʱ��5����ʱ������1S
	
	//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )EVENT_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )INTERRUPT_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    EventGroupHandle = xEventGroupCreate();
    //�����¼��鴦������
    xTaskCreate((TaskFunction_t )event_task,  			//������
                (const char*    )"event_task", 			//��������
                (uint16_t       )INTERRUPT_STK_SIZE,		//�����ջ��С
                (void*          )NULL,						//���ݸ��������Ĳ���
                (UBaseType_t    )INTERRUPT_TASK_PRIO,		//�������ȼ�
                (TaskHandle_t*  )&EVENTTASK_Handler); 	//������
								
    xTaskCreate((TaskFunction_t )temp_task,  			//������
                (const char*    )"temp_task", 			//��������
                (uint16_t       )TEMP_STK_SIZE,		//�����ջ��С
                (void*          )NULL,						//���ݸ��������Ĳ���
                (UBaseType_t    )TEMP_TASK_PRIO,		//�������ȼ�
                (TaskHandle_t*  )&TEMPTask_Handler); 	//������


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
    //����LED����
    xTaskCreate((TaskFunction_t )LED_task,  			//������
                (const char*    )"LED_task", 			//��������
                (uint16_t       )LED_STK_SIZE,		//�����ջ��С
                (void*          )NULL,						//���ݸ��������Ĳ���
                (UBaseType_t    )LED_TASK_PRIO,		//�������ȼ�
                (TaskHandle_t*  )&LEDTask_Handler); 	//������	
    //�����жϲ�������
    xTaskCreate((TaskFunction_t )interrupt_task,  			//������
                (const char*    )"interrupt_task", 			//��������
                (uint16_t       )INTERRUPT_STK_SIZE,		//�����ջ��С
                (void*          )NULL,						//���ݸ��������Ĳ���
                (UBaseType_t    )INTERRUPT_TASK_PRIO,		//�������ȼ�
                (TaskHandle_t*  )&INTERRUPTTask_Handler); 	//������
    //����QUERY����
    xTaskCreate((TaskFunction_t )query_task,     
                (const char*    )"query_task",   
                (uint16_t       )QUERY_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )QUERY_TASK_PRIO,
                (TaskHandle_t*  )&QueryTask_Handler); 

	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	
  taskEXIT_CRITICAL();            //�˳��ٽ���
}


void temp_task(void *pvParameters)
{
	static short tempera =0;
	while(1)
	{
		tempera = DS18B20_Get_Temp();
		printf("Update: current temperature is %.1f\r\n", (float)tempera/10);
		Readburst(gSensor.Temperaturer.m_Time);
		printf("Time:20%d��%d��%d��%dʱ%d��%d��\r\n",	gSensor.Temperaturer.m_Time[6],
																									gSensor.Temperaturer.m_Time[4],
																									gSensor.Temperaturer.m_Time[3],
																									gSensor.Temperaturer.m_Time[2],
																									gSensor.Temperaturer.m_Time[1],
																									gSensor.Temperaturer.m_Time[0]);
		vTaskDelay(10000);
	}

}

//�жϲ��������� 
void interrupt_task(void *pvParameters)
{
	static u32 total_num=0;
    while(1)
    {
		total_num+=1;
		if(total_num==5) 
		{
			printf("�ر��ж�.............\r\n");
			total_num = 0;
//			portDISABLE_INTERRUPTS();				//�ر��ж�
			delay_xms(50000);						//��ʱ5s
			printf("���ж�.............\r\n");	//���ж�
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
