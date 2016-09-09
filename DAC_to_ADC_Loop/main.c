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
#include "tm_stm32f4_hd44780.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
#define DACBUFFERSIZE 		256
#define WAVEFREQ			1 /* 1KHz */
#define TIMER6_PRESCALER	32	/* produces a 42MHz tick */
#define TIMER_CLOCK			84E6

uint16_t DACBuffer1[DACBUFFERSIZE];
void RCC_Configuration(void);
void ADC_Configuration(void);
int read_ADC(void);
void RCC_Configuration(void);
void DMA_Configuration( uint16_t* wavBuffer );

void GPIO_Configuration(void);

void Timer_Configuration(uint16_t wavPeriod, uint16_t preScaler);
void DAC_Configuration(void);
void delay_ms(uint32_t milli);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	uint32_t fTimer;
	uint32_t timerFreq;
	uint16_t timerPeriod;
	uint16_t n;
	uint16_t m;
	uint32_t buttonVal;
	uint8_t buffNum = 1;
	for (n = 0; n<DACBUFFERSIZE; n++)
	{
		if(n<DACBUFFERSIZE/2)
			DACBuffer1[n]=0;
		else
			DACBuffer1[n]=4095;
	}
	/* Calculate frequency of timer */
	fTimer = WAVEFREQ * DACBUFFERSIZE;

	/* Calculate Tick Rate */
	timerFreq = TIMER_CLOCK / TIMER6_PRESCALER; /* Timer tick is in Hz */

	/* Calculate period of Timer */
	timerPeriod = (uint16_t)( timerFreq / fTimer );
  ADC_Configuration();
  RCC_Configuration();
  GPIO_Configuration();
  Timer_Configuration(timerPeriod, TIMER6_PRESCALER);
  DAC_Configuration();
  DMA_Configuration(&DACBuffer1);
  TM_HD44780_Init(16,2);
  TM_HD44780_DisplayOn();
  TM_HD44780_CursorOff();
  int ADC_Value=0;
  int log_map;
  int ADC_Percent;
  float logValue;
  uint16_t Previous_Value=0;
  char volume[3];
  while (1)
  {
	  ADC_Value=read_ADC();

	  sprintf(volume, "%d", ADC_Value);
	  if(ADC_Value>(Previous_Value+30)||ADC_Value<(Previous_Value-30)){
		  TM_HD44780_Clear();
		  TM_HD44780_Puts(0,0,"DAC: ");
		  TM_HD44780_Puts(9,0,volume);
		  Previous_Value=ADC_Value;
	  }



  }
}
void RCC_Configuration(void)
{
	/* Enable DMA and GPIOA Clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_GPIOD, ENABLE);

	/* Enable DAC1 and TIM6 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM6, ENABLE);
}
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Pack the struct */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/**
  * @brief  Configures the DMA.
  * @param  None
  * @retval : None
  */
void DMA_Configuration( uint16_t* wavBuffer )
{
	DMA_InitTypeDef DMA_InitStructure;

	//Initialize the structure to default values
	DMA_StructInit(&DMA_InitStructure);

	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(DAC_BASE + 0x08);  //DAC channel1 12-bit right-aligned data holding register (ref manual pg. 264)
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)wavBuffer;
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

	/* Enable DMA */
	DMA_Cmd(DMA1_Stream5, ENABLE);

}

/**
  * @brief  Configures the Timers.
  * @param  wavePeriod (period of timer), preScaler (prescaler for timer)
  * @retval : None
  */
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

/**
  * @brief  Configures the DAC
  * @param  None
  * @retval : None
  */
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
void ADC_Configuration(){
	 ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
	 GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	 //Clock configuration
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
	 RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN,ENABLE);//Clock for the ADC port!! Do not forget about this one ;)
	 //Analog pin configuration
	 GPIO_initStructre.GPIO_Pin = GPIO_Pin_0;//The channel 10 is connected to PC0
	 GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	 GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	 GPIO_Init(GPIOC,&GPIO_initStructre);//Affecting the port with the initialization structure configuration
	 //ADC structure configuration
	 ADC_DeInit();
	 ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
	 ADC_init_structure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
	 ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	 ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
	 ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
	 ADC_init_structure.ADC_NbrOfConversion = 1;//I think this one is clear :p
	 ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
	 ADC_Init(ADC1,&ADC_init_structure);//Initialize ADC with the previous configuration
	 //Enable ADC conversion
	 ADC_Cmd(ADC1,ENABLE);
	 //Select the channel to be read from
	 ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_144Cycles);
}
int read_ADC(void){
	ADC_SoftwareStartConv(ADC1);//Start the conversion
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	return ADC_GetConversionValue(ADC1); //Return the converted data
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
