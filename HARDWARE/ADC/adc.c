#include "adc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "header.h"
#include "FreeRTOS.h"
#include "event_groups.h"

void MBDMA_Init(void);
void Get_ADC_Value(void);

void Infra_Init(void)
{	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE ); 
	

	RCC_ADCCLKConfig(RCC_PCLK2_Div2);	//f(ADC) = 12 MHz

					  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
	ADC_InitStructure.ADC_NbrOfChannel = NumberOfSamplingChannel;	
	ADC_Init(ADC1, &ADC_InitStructure); 


 	 /* setup the regular channel for ADC1*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5 );		//PA1 == Channel_1 == Infra-1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_7Cycles5 );	  // PA4 =	Channel_4 = Infra-2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_7Cycles5 );		//PA3 = Channel_3 = CAN_ID
	/* setup the injection channel for ADC1*/
	ADC_InjectedSequencerLengthConfig(ADC1, 1);
	ADC_InjectedChannelConfig(ADC1,ADC_Channel_3,1,ADC_SampleTime_239Cycles5);
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	
	
	ADC_Cmd(ADC1, ENABLE);	//?????ADC1
	
	MBDMA_Init();
	
	ADC_DMACmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1); 
	 
	while(ADC_GetResetCalibrationStatus(ADC1)); 
	
	ADC_StartCalibration(ADC1);  
 
	while(ADC_GetCalibrationStatus(ADC1));	 
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
//	ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
//	value1 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
}

void MBDMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure; 
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//??DMA??
	
  	DMA_DeInit(DMA1_Channel1);   //?DMA???1?????????
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; ;  //DMA?????
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) &gSensor.RawData[0];  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	
	DMA_InitStructure.DMA_BufferSize = NumberOfSamplingChannel;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //U16
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //U16
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); 
	
	
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);	
	NVIC_InitStructure.NVIC_IRQChannel	=DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	= 0;   //
	NVIC_InitStructure.NVIC_IRQChannelCmd	= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void DMA1_Channel1_IRQHandler(void)
{
//	BaseType_t Result, xHigherPriorityTaskWoken ;
//	xHigherPriorityTaskWoken = pdFALSE;
   if(DMA_GetITStatus(DMA1_IT_TC1))
	{  
		DMA_Cmd(DMA1_Channel1, DISABLE);
		Get_ADC_Value();
//		Result = xEventGroupSetBitsFromISR(EventGroupHandle,INFRA_EVENTBIT,&xHigherPriorityTaskWoken);
//		if(Result != pdFAIL)
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		DMA_SetCurrDataCounter(DMA1_Channel1,NumberOfSamplingChannel);
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		DMA_ClearFlag(DMA1_IT_TC1);   
	}
		DMA_Cmd(DMA1_Channel1, ENABLE);
}

/*
1. calculate the voltage of infrared sensors
2. unit: mV
*/

void Get_ADC_Value(void)
{
	gSensor.InfraredSensor.Infra01.m_Distance = gSensor.RawData[0]*81/100; 
	gSensor.InfraredSensor.Infra02.m_Distance = gSensor.RawData[1]*81/100; 
}


