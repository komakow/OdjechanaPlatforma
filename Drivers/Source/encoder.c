/*
 *@encoder.c
 *
 *@Created on: 12.04.2018
 *@Author: KamilM
 *
 *@brief Source code of encoder driver
 */

#ifndef ENCODER_C_
#define ENCODER_C_

//includes
#include "stm32f4xx.h"
#include "encoder.h"
#include "defGPIO.h"
#include "GPIODriver.h"
#include "uart.h"

volatile float speed_encoder = 0;

extern UARTcfg UART;                                                                       //uart config scructure

static err TIM5_Init()
{
  err ret = 0;

  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;						      //start clock for TIM5
  TIM5->SMCR = TIM_SMCR_SMS_1;							          //count only TI2
  TIM5->CCMR1 = TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;			//filtr
  TIM5->ARR = 6;
  TIM5->DIER = TIM_DIER_UIE;
  TIM5->CR1 = TIM_CR1_CEN;

  return ret;
}

static err TIM6_Init()
{
  err ret = 0;

  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;						//start clock for TIM6

  TIM6->DIER = TIM_DIER_UIE;								    //enabled update interrupt
  TIM6->CNT = 0;
  TIM6->PSC = 1000;										          //one tick for 0,1ms
  TIM6->CR1 = TIM_CR1_CEN;
  return ret;
}

err Encoder_Init()
{
  err ret = 0;

  TIM5_Init();											            //init and enable TIM5
  TIM6_Init();											            //init and enable TIM6
  Encoder_IRQ_Enable();									        //enable interrupts

  return ret;
}

err Encoder_IRQ_Enable()
{
  err ret = 0;

  //TIM5
  NVIC_ClearPendingIRQ(TIM5_IRQn);						                    //clear pending interrupts
  NVIC_EnableIRQ(TIM5_IRQn);								                      //enable interrupt

  //TIM6
  NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);						              //clear pending interrupts
  NVIC_EnableIRQ(TIM6_DAC_IRQn);								                //enable interrupt

  return ret;
}


err Encoder_IRQ_Disable()
{
  err ret = 0;
  //TIM5
  NVIC_ClearPendingIRQ(TIM5_IRQn);                                //clear pending interrupts
  NVIC_DisableIRQ(TIM5_IRQn);                                      //enable interrupt

  //TIM6
  NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);                          //clear pending interrupts
  NVIC_DisableIRQ(TIM6_DAC_IRQn);                                //enable interrupt
  return ret;
}


float Encoder_SpeedSend()
{
  uint8_t sendSpeed[4];
  sendSpeed[0] = 'V';                                           //char started frame of velocity
  sendSpeed[1] = (uint8_t)speed_encoder;
  sendSpeed[2] = 10;
  sendSpeed[3] = 13;
  UART_Send(sendSpeed, 4, &UART);

  return speed_encoder;
}

__attribute__((interrupt)) void TIM5_IRQHandler(void)         //interrupt form encoder
{

  int stop = 0;
  if (TIM5->SR & TIM_SR_UIF)
  {
    stop = TIM6->CNT;
    speed_encoder = (0.15 / stop) * 42000;                            // cm/s
    TIM6->CNT = 0;
    TIM5->SR = ~TIM_SR_UIF;                                   //clear update irq flag

  }
}

__attribute__((interrupt)) void TIM6_DAC_IRQHandler(void)     //interrupt when encoder idle for over 1sec
{
  TIM6->SR = ~TIM_SR_UIF;                                     //clear update irq flag
  speed_encoder = 0;
}

#endif /* ENCODER_C_ */
