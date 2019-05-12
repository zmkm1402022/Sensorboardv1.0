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
#include "inv_mpu.h"
#include "data_builder.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

struct int_param_s int_param;
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};

#define COMPASS_ENABLED 0

#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif

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
struct int_param_s int_param;
u8 chk;

volatile uint32_t g_ul_ms_ticks=0;
u8 msg_IMU;


static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
    case TAP_X_UP:
        printf("Tap X+ ");
        break;
    case TAP_X_DOWN:
        printf("Tap X- ");
        break;
    case TAP_Y_UP:
        printf("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        printf("Tap Y- ");
        break;
    case TAP_Z_UP:
        printf("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        printf("Tap Z- ");
        break;
    default:
        return;
    }
    printf("x%d\n", count);
    return;
}

static void android_orient_cb(unsigned char orientation)
{
	switch (orientation) {
	case ANDROID_ORIENT_PORTRAIT:
        printf("Portrait\n");
        break;
	case ANDROID_ORIENT_LANDSCAPE:
        printf("Landscape\n");
        break;
	case ANDROID_ORIENT_REVERSE_PORTRAIT:
        printf("Reverse Portrait\n");
        break;
	case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        printf("Reverse Landscape\n");
        break;
	default:
		return;
	}
}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 1);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
	printf("Passed!\n");
        printf("accel: %7.4f %7.4f %7.4f\r\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        printf("gyro: %7.4f %7.4f %7.4f\r\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
     /* Push the calibrated data to the MPL library.
     *
     * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
    	unsigned short accel_sens;
    	float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
            if (!(result & 0x1))
                printf("Gyro failed.\r\n");
            if (!(result & 0x2))
                printf("Accel failed.\r\n");
            if (!(result & 0x4))
                printf("Compass failed.\r\n");
     }

}


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
	WHDG_IO = ~WHDG_IO;
	I2cMaster_Init();
	Set_I2C_Retry(5);
	msg_IMU = mpu_init(&int_param);
	if(msg_IMU == 0)
		printf("Note: IMU starts up successfully\r\n");
	else
		printf("Error: IMU fails to startup\r\n");
	DMPSetup();
	msg_IMU = dmp_load_motion_driver_firmware();
  msg_IMU= dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
  msg_IMU = dmp_register_tap_cb(tap_cb);
  msg_IMU = dmp_register_android_orient_cb(android_orient_cb);
  hal.dmp_features = DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP| DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO| DMP_FEATURE_GYRO_CAL;
  msg_IMU = dmp_enable_feature(hal.dmp_features);
  msg_IMU = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	run_self_test();
  msg_IMU = mpu_set_dmp_state(1);
    hal.dmp_on = 1;
//	for (msg_IMU =0; msg_IMU < 20; msg_IMU++){
//		gSensor.imuData.ID = 0;
//		Sensors_I2C_ReadRegister(0x68, 0x75, 1, &gSensor.imuData.ID);
//		printf("failt to get  gyro data %x\r\n",gSensor.imuData.ID);
//		delay_ms(1000);
//		
//	}
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
		taskENTER_CRITICAL();  
		tempera = DS18B20_Get_Temp();
		printf("Update: current temperature is %.1f\r\n", (float)tempera/10);
		Readburst(gSensor.Temperaturer.m_Time);
		printf("Time:20%d年%d月%d日%d时%d分%d秒\r\n",	gSensor.Temperaturer.m_Time[6],
																									gSensor.Temperaturer.m_Time[4],
																									gSensor.Temperaturer.m_Time[3],
																									gSensor.Temperaturer.m_Time[2],
																									gSensor.Temperaturer.m_Time[1],
																									gSensor.Temperaturer.m_Time[0]);
//		tempera = dmp_read_fifo(gSensor.imuData.gyro, gSensor.imuData.accel_short,gSensor.imuData.quat,&gSensor.imuData.sensor_timestamp, &gSensor.imuData.sensors, &gSensor.imuData.more);
		
		taskEXIT_CRITICAL(); 

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
		gSensor.imuData.ID = 0;
		Sensors_I2C_ReadRegister(0x68, 0x75, 1, &gSensor.imuData.ID);
		printf("error: fail to get the data from IMU, ID = %x\r\n",gSensor.imuData.ID );		
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
