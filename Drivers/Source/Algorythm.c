/**
 *@Algorythm.h
 *
 *@Created on: 30.05.2018
 *@Author: KamilM
 *
 *@brief Source code of function used to control main algorithm of 'odjechana platforma'
 */
#include "main.h"

const uint8_t CR = 13;                                                    // /r char send at last byte at frame
const uint8_t LF = 10;                                                    // /n char send at last byte at frame

                                                             //buffor to storage data read from UART

int PeripheryInit()
{
  //Status of error
  int error=0;

  /*
   * readSpeed[0] - Car forward speed
   * readSpeed[1] - Car reverse speed
   * readSpeed[2] - Car turn left
   * readSpeed[3] - Car turn right
   */
  uint8_t readSpeed[4];

  //***************UART Init*************************

  UART.uart = UART_3;
  UART.baundRate = 28800;                                                                //in real it is 9600 baund (error in hardware stm?)
  UART.parityControl = unused;
  UART.readLength = 4;                                                                    //length read frame form BT
  UART.readMemory = readSpeed;

  error += UART_Init(&UART);
  UART_Read(&UART);                                                                       //start read

  //***************HBridge Init**********************
  error += HBridge_Init();

  //***************HC_SR04 sonic sensor Init*********
  HC_SR04_Init();

  //***************Encoder Init*********************
  error += Encoder_Init();

  //**************External interrupt*****************
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  __DSB();
  SYSCFG->EXTICR[2] |=SYSCFG_EXTICR2_EXTI4_PA;                                              //external interrupt from PA4
  EXTI->FTSR |= EXTI_FTSR_TR4;                                                              //falling edge at PA4
  EXTI->RTSR |= EXTI_RTSR_TR4;                                                              //rising edge at PA4
  EXTI->IMR |= EXTI_IMR_MR4;                                                                //interrupt request is not masked
  NVIC_EnableIRQ(EXTI4_IRQn);                                                               //enable interrupt at line 4

  return error;
}


void BTDisconnected()
{
  WheelStop();                                  //stop all wheel
}


void delay()
{
  for(int a = 0;a<10;a++)
  {
    for(int b=0;b<40000;b++)
    {
      __NOP();
    }
  }
}

void delay_run()
{
  for(int a = 0;a<30;a++)
  {
    for(int b=0;b<500;b++)
    {
      __NOP();
    }
  }
}


int RunCar()
{
  int ret = 0;
  uint8_t left;
  uint8_t right;

  uint8_t* readdSpeed_ptr = (uint8_t*)DMA1_Stream1->M0AR;            //pointner to first byte of read speed from UART
  delay();
  uint8_t readSpeed[4];
  readSpeed[0] = readdSpeed_ptr[0];
  readSpeed[1] = readdSpeed_ptr[1];
  readSpeed[2] = readdSpeed_ptr[2];
  readSpeed[3] = readdSpeed_ptr[3];


  if( (readSpeed[0] != 0) )                                     //ride forward
  {

    if( readSpeed[2] > readSpeed[0] )                           //check turn forward
    {
      left = 0;
    }
    else
    {
      left = readSpeed[0] - readSpeed[2];                       //variable represent speed of left wheel
    }


    if( readSpeed[3] > readSpeed[0] )                           //check turn reverse
    {
      right = 0;
    }
    else
    {
      right = readSpeed[0] - readSpeed[3];                      //variable represent speed of right wheel
    }

    WheelControl(Left, Forward, left);
    WheelControl(Right, Reverse, right);
  }


  else if( (readSpeed[1] != 0) )                                //ride reverse
  {
    if( readSpeed[2] > readSpeed[1] )                           //check turn forward
    {
      left = 0;
    }
    else
    {
      left = readSpeed[1] - readSpeed[2];                       //variable represent speed of left wheel
    }



    if( readSpeed[3] > readSpeed[1] )                         //check turn reverse
    {
      right = 0;
    }
    else
    {
      right = readSpeed[1] - readSpeed[3];                  //variable represent speed of right wheel
    }


    WheelControl(Left, Reverse, left);
    WheelControl(Right, Forward, right);
  }
  else if( (readSpeed[0] == 0) && (readSpeed[1] == 0) && (readSpeed[0] == 0) && (readSpeed[1] == 0) )
  {
    WheelStop();
  }

  return ret;
}


uint8_t DistanceReadSend(uint8_t* distance)
{

  //left sonic sensor
  uint8_t sendSonicL[4];
  float sonicL;

  //right sonic sensor
  uint8_t sendSonicR[4];
  float sonicR;

  //center sonic sensor
  uint8_t sendSonicC[4];
  float sonicC;

  //back sonic sensor
  uint8_t sendSonicB[4];
  float sonicB;

  //Left sonic sensor
  sonicL = HC_SR04_ReadDistance(HC_left);
  sendSonicL[0] = 'L';                                           //char started frame of left sensor
  sendSonicL[1] = (uint8_t)sonicL;
  sendSonicL[2] = CR;
  sendSonicL[3] = LF;
  UART_Send(sendSonicL, 4, &UART);                               //send by UART

  //delay_run();
  //Right sonic sensor
  sonicR = HC_SR04_ReadDistance(HC_right);
  sendSonicR[0] = 'R';                                           //char started frame of right sensor
  sendSonicR[1] = (uint8_t)sonicR;
  sendSonicR[2] = CR;
  sendSonicR[3] = LF;
  UART_Send(sendSonicR, 4, &UART);                               //send by UART

  //delay_run();
  //Center sonic sensor
  sonicC = HC_SR04_ReadDistance(HC_center);
  sendSonicC[0] = 'C';                                           //char started frame of center sensor
  sendSonicC[1] = (uint8_t)sonicC;
  sendSonicC[2] = CR;
  sendSonicC[3] = LF;
  UART_Send(sendSonicC, 4, &UART);                               //send by UART

  //delay_run();
  //Back sonic sensor
  sonicB = HC_SR04_ReadDistance(HC_back);
  sendSonicB[0] = 'B';                                           //char started frame of back sensor
  sendSonicB[1] = (uint8_t)sonicB;
  sendSonicB[2] = CR;
  sendSonicB[3] = LF;
  UART_Send(sendSonicB, 4, &UART);                               //send by UART

  distance[0] = (uint8_t) sonicL;
  distance[1] = (uint8_t) sonicC;
  distance[2] = (uint8_t) sonicR;
  distance[3] = (uint8_t) sonicB;


  return 0;
}
