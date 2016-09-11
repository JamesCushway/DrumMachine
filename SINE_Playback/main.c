/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed "as is", without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. This file may only be built (assembled or compiled and linked)
**  using the Atollic TrueSTUDIO(R) product. The use of this file together
**  with other tools than Atollic TrueSTUDIO(R) is not permitted.
**
*****************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#define DACBUFFERSIZE 		500
#define WAVEFREQ			500 /* 1KHz */
#define TIMER6_PRESCALER	2	/* produces a 42MHz tick */
#define TIMER_CLOCK			84E6
uint16_t DACBuffer[DACBUFFERSIZE];
uint16_t DACBuffer2[DACBUFFERSIZE];
uint8_t record=0;
uint8_t playback=0;

void RCC_Configuration();
void GPIO_Configuration();
void Configure_PD0();
void Configure_PD1();
void Timer_Configuration(uint16_t wavPeriod, uint16_t preScaler);
void DAC_Configuration();
void ADC_Configuration();
void DMA_Configuration();
void DMA_Configuration2(uint32_t* Buffer);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET ) {
    	while(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0));
		if(record==0){
			ADC_Configuration();
			DMA_Configuration();

			record=1;
		}
		else if(record==1){
			DMA_Cmd(DMA2_Stream4,DISABLE);
			record=0;
		}
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/* Handle PB12 interrupt */
void EXTI15_10_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
    	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12));
        /* Do your stuff when PB12 is changed */
		if(playback==0&&record==0){
			DMA_Cmd(DMA1_Stream5,DISABLE);
			DMA_Configuration2(&DACBuffer2);
			playback=1;
		}
		else if(playback==1){
			DMA_Cmd(DMA1_Stream5,DISABLE);
			DMA_Configuration2(&DACBuffer);
			playback=0;
		}

        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
int main(void)
{
	uint32_t fTimer;
	uint32_t timerFreq;
	uint16_t timerPeriod;
	uint16_t n;

	for (n = 0; n<DACBUFFERSIZE; n++)
	{
		DACBuffer[n] = (uint16_t)(((4096)/2)*(sin(2*3.14*n/DACBUFFERSIZE)+1));
	}
	/* Calculate frequency of timer */
	fTimer = WAVEFREQ * DACBUFFERSIZE;
	/* Calculate Tick Rate */
	timerFreq = TIMER_CLOCK / TIMER6_PRESCALER; /* Timer tick is in Hz */
	/* Calculate period of Timer */
	timerPeriod = (uint16_t)( timerFreq / fTimer );


	RCC_Configuration();
	GPIO_Configuration();
	Configure_PD0();
	Configure_PB12();
	Timer_Configuration(timerPeriod, TIMER6_PRESCALER);
	DAC_Configuration();
	DMA_Configuration2((uint32_t*)&DACBuffer);


	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	/* Infinite loop */
	while (1)
	{
		if(record==1){
			STM_EVAL_LEDOff(LED6);
			STM_EVAL_LEDToggle(LED5);
			for(int i=0;i<3000000;i++);
		}
		else if(playback==1){
			STM_EVAL_LEDOff(LED5);
			STM_EVAL_LEDToggle(LED6);
			for(int i=0;i<3000000;i++);
		}
		else{
			STM_EVAL_LEDOff(LED5);
			STM_EVAL_LEDOff(LED6);
		}
	}
}


void RCC_Configuration(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_DMA1|RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC|RCC_APB1Periph_TIM6, ENABLE);
}

void Timer_Configuration(uint16_t wavPeriod, uint16_t preScaler)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;

	/* pack Timer struct */
	TIM_TimeBaseStruct.TIM_Period = wavPeriod-1;
	TIM_TimeBaseStruct.TIM_Prescaler = preScaler-1;
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0x0000;

	/* Call init function */
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);

	/* Select Timer to trigger DAC */
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);
}
void Configure_PD0(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

void Configure_PB12(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOB */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Tell system that you will use PB12 for EXTI_Line12 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);

    /* PB12 is connected to EXTI_Line12 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line12;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

void DMA_Configuration(){
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA2_Stream4);  //Set DMA registers to default values
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //Source address
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DACBuffer2; //Destination address
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 100; //Buffer size
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure); //Initialize the DMA
	DMA_Cmd(DMA2_Stream4,ENABLE);
}

void DMA_Configuration2(uint32_t* Buffer){
	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(DAC_BASE + 0x08);  //DAC channel1 12-bit right-aligned data holding register (ref manual pg. 264)
	DMA_InitStructure.DMA_Memory0BaseAddr = Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = DACBUFFERSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	/* Call Init function */
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5,ENABLE);

}

void GPIO_Configuration(){
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void ADC_Configuration(){
	ADC_InitTypeDef       ADC_InitStructure;

	ADC_StructInit(&ADC_InitStructure);


	ADC_DeInit();
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_InitStructure.ADC_ScanConvMode =DISABLE; // 1=scan more that one channel in group
	ADC_Init(ADC1,&ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,1,ADC_SampleTime_480Cycles);


	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA

	ADC_Cmd(ADC1, ENABLE);   // Enable ADC1

	ADC_SoftwareStartConv(ADC1); // Start ADC1 conversion
}

void DAC_Configuration(void)
{
	DAC_InitTypeDef DAC_InitStruct;

	/* Initialize the DAC_Trigger member */
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;
	/* Initialize the DAC_WaveGeneration member */
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	/* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	/* Initialize the DAC_OutputBuffer member */
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	/* Init DAC */
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);

	/* Enable DMA request */
	DAC_DMACmd(DAC_Channel_1, ENABLE);

	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);

}
/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
