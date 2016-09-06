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
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

void RCC_Configuration(void);
void ADC_Configuration(void);
int read_ADC(void);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{

  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  ADC_Configuration();
  TM_HD44780_Init(16,2);
  TM_HD44780_DisplayOn();
  TM_HD44780_CursorOff();
  int ADC_Value=0;
  uint16_t Previous_Value=0;
  char volume[3];
  while (1)
  {
	  ADC_Value=read_ADC();

	  sprintf(volume, "%d", (ADC_Value*100)/4095);
	  if(ADC_Value>(Previous_Value+10) || ADC_Value<(Previous_Value-10)){
		  TM_HD44780_Clear();
		  TM_HD44780_Puts(0,0,"Volume: ");
		  TM_HD44780_Puts(9,0,volume);
	  }
  }
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
