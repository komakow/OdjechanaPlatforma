/*
 *@HC-SR04.c
 *
 *@Created on: 07.05.2018
 *@Author: Kamil Malski
 *
 *@brief Source code of ultrasonic sensor driver used by "Odjechana Platforma" project
 */

#include "HC_SR04.h"
#include "stm32f4xx.h"
#include "defGPIO.h"

int overtime = 49980;										//about 50ms

static void delay_us(int delay)
{
	for(int b=0;b<delay;b++)
	{
		for(int a=0;a<4;a++)								//I guess it is 1 us
		{
			__NOP();
		}
	}
}

static float ReadDistance()
{
	int capture_time = 0;
	float distance = 0;

	TIM10->CNT = 0;											              //clear CNT
	TIM10->SR &=~TIM_SR_UIF;
	while( !(TIM10->SR & TIM_SR_UIF) )					      //wait until capture
	{
		if( TIM10->CNT > overtime - 1 )							    //if timer is overflow
			break;
	}

	capture_time = TIM10->CCR1;								    //save capture value of timer


	distance = ((capture_time) * 34 ) / 2000;			//from datasheet HC-SR04

	return distance;										          //return distance in [cm]
}


void HC_SR04_Init()
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;						   //enable clock for TIM10

  TIM10->CCMR1 |= TIM_CCMR1_CC1S_0;						      //CC1 channel is input and mapped at TI1
  TIM10->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;			//enable capture, falling edge
  TIM10->PSC = 84;										              //prescaler, one tick CNT is one uS
  TIM10->ARR = overtime;									          //50k
  TIM10->CR1 |= TIM_CR1_CEN;								        //start timer

}

float HC_SR04_ReadDistance(HCSRTypeDef sensor)
{

  float distance = 0;

  switch (sensor)
  {
    case HC_left :
    {
      gpioPinSetState(PORTE, PIN0, 1);				//set high trigger of sensor
      delay_us(10);										              //delay 10us
      gpioPinSetState(PORTE, PIN0, 0);				//set low trigger of sensor

      distance = ReadDistance();							      //read distance in [cm]

      break;
    }

    case HC_center :
    {
      gpioPinSetState(PORTE, PIN1, 1);			//set high trigger of sensor
      delay_us(10);										              //delay 10us
      gpioPinSetState(PORTE, PIN1, 0);			//set low trigger of sensor

      distance = ReadDistance();							      //read distance in [cm]

      break;
    }

    case HC_right :
    {
      gpioPinSetState(PORTE, PIN2, 1);				//set high trigger of sensor
      delay_us(10);										              //delay 10us
      gpioPinSetState(PORTE, PIN2, 0);				//set low trigger of sensor

      distance = ReadDistance();							      //read distance in [cm]

      break;
    }
    case HC_back :
    {
      gpioPinSetState(PORTE ,PIN3, 1);				//set high trigger of sensor
      delay_us(10);										              //delay 10us
      gpioPinSetState(PORTE ,PIN3, 0);				//set low trigger of sensor

      distance = ReadDistance();							      //read distance in [cm]

      break;
    }

    default :
      distance = -1;
      break;
  }

  return distance;										            //return distance in [cm]
}

void HC_SR04_DeInit()
{
	TIM10->CR1 &=~TIM_CR1_CEN;								      //disable TIM10
	TIM10->CNT = 0;											            //clear CNT
	RCC->APB2ENR &=~RCC_APB2ENR_TIM10EN;					  //disable clock for TIM10

}
