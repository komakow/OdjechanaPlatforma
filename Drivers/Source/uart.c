/*
 *@ file uart.c
 *
 *@brief source code of UART controller write in C, work at stm32f407vg
 *
 *  Created on: 17.05.2017
 *      Author: malskik
 */
//************************************************INCLUDES**********************************

#include "stm32f4xx.h"
#include "uart.h"


//********************************************GLOBAL VARIABLES******************************

void (*ApplicationHandler3)(void);
uint32_t is_init = 0;                                                                //flag checked UART_Init status
uint32_t countByte = 0;                                                              //variable used by UART_CountByte()
uint32_t countCircle = 0;                                                            //vairable used by UART_CountByte()


//*********************************************STATIC FUNCTION*******************************

static int Config_Clock_UART3()
{
  int ret;

  //***********************CONFIG_CLOCK_PORT_AND_PIN************************
  RCC->APB1ENR |=RCC_APB1ENR_USART3EN;                                               //clock for USART3
  RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;                                                  //clock for DMA1

  if((RCC->APB1ENR & RCC_APB1ENR_USART3EN) && (RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN))   //check correctness of the settings
  {
    ret = E_UART_OK;
  }
  else
    ret = E_UART_NOT_INITIALIZED;

  return ret;
}

static int Config_DMA_UART3()
{
  int ret;
  //****************************DMA1_RX************************************
  DMA1_Stream1->PAR = (uint32_t)&USART3->DR;										                  //peripheral address
  DMA1_Stream1->CR |= DMA_SxCR_CHSEL_2;	                                          //set channel_4, request form USART3
  DMA1_Stream1->CR |= DMA_SxCR_MINC | DMA_SxCR_CIRC;								              //8-bit word, memory increment mode, read from peripheral

  //****************************DMA2_TX************************************
  DMA1_Stream3->PAR = (uint32_t)& USART3->DR;									                	//peripheral address
  DMA1_Stream3->CR |= DMA_SxCR_CHSEL_2;		                                      //set channel_4, request form USART3
  DMA1_Stream3->CR |= DMA_SxCR_MINC | DMA_SxCR_DIR_0;							            	//8-bit word, memory increment mode,read from memory

  ret = E_UART_OK;
  return ret;
}

static int Config_UART3(UARTcfg* config)
{
  int ret;
  uint32_t div;
  //*****************************USART1************************************
  switch(config->baundRate)
  {
    case 9600:
      div=42000000/(config->baundRate);
      break;
    case 14400:
      div=42000000/(config->baundRate);
      break;
    case 19200:
      div=42000000/(config->baundRate);
      break;
    case 28800:
      div=42000000/(config->baundRate);
      break;
    case 38400:
      div=42000000/(config->baundRate);
      break;
    case 56000:
      div=42000000/(config->baundRate);
      break;
    case 57600:
      div=42000000/(config->baundRate);
      break;
    case 115200:
      div=42000000/(config->baundRate);
      break;
    default:
      ret = E_UART_INVALID_PARAM;
      break;
  }

  if(config->parityControl == used)
  {
    USART3->CR1 |= USART_CR1_PCE;                                                                                //set even parity control
    USART3->CR1 |= USART_CR1_M;                                                                                  //9 bit data
  }

  //USART3->BRR= 0x1117;
  USART3->BRR= (div & 0xfff0) | (div & 0xf) >> 1;                                                                //set baudrate   TODO: !!!!!!!!!!!!!!!!!!!Zjebany baudrate!!!!!!!!!!!!!!!
  USART3->CR1 |= USART_CR1_RE | USART_CR1_TE;                                                                    //enable receiver, enable transmitter
  USART3->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;                                                                //DMA enable receiver and transmitter
  USART3->CR1 |= USART_CR1_UE;                                                                                   //enable UART


  if((USART3->CR1 & USART_CR1_UE) && (USART3->CR1 & USART_CR1_RE) && (USART3->CR1 & USART_CR1_TE))              //check correctness of the setting CR1 register
  {
    if((USART3->CR3 & USART_CR3_DMAR) && (USART3->CR3 & USART_CR3_DMAT))                                        //check correctness of the setting CR3 register
    {
      ret = E_UART_OK;
    }
    else
      ret = E_UART_NOT_INITIALIZED;
  }
  else
    ret = E_UART_NOT_INITIALIZED;

  return ret;
}

static int Send_UART3(uint8_t *data_send,uint32_t send_length, UARTcfg* config)
{
  int ret;
  while( !(USART3->SR & USART_SR_TC) );

  DMA1_Stream3->CR &= ~DMA_SxCR_EN;												//DMA stop
  DMA1_Stream3->NDTR = send_length;												//number of 8-bit words
  DMA1_Stream3->M0AR = (uint32_t)data_send;               //address of memory with data to send
  DMA1->LIFCR |= DMA_LIFCR_CTCIF3;                        //clear transfer complete interrupt
  USART3->SR &=~USART_SR_TC;                              //clear transmission complete flag
  DMA1_Stream3->CR |= DMA_SxCR_EN;												//start DMA

  ret=E_UART_OK;

 return ret;
}

static int Read_UART3(UARTcfg* config)
{
  int ret;

  DMA1_Stream1->CR &= ~DMA_SxCR_EN;												                         //DMA stop
  DMA1_Stream1->NDTR = config->readLength;                                        //place in memory to save data (circle mode)
  DMA1_Stream1->M0AR = (uint32_t)config->readMemory;                              //pointner to the first byte in memory
  DMA1_Stream1->CR |= DMA_SxCR_EN;												                        //DMA start

  ret = E_UART_OK;

  return ret;
}

static uint8_t* Pointner_UART3(UARTcfg* config)
{
  uint32_t actualValueCNDTR = DMA1_Stream3->NDTR;                                   //actual value of NDTR register
  uint32_t userValueCNDTR = config->readLength;                                     //initial value of CNDTR

  uint32_t passedData = userValueCNDTR - actualValueCNDTR;                          //number of passed data

  uint8_t* startAddress = (uint8_t*)config->readMemory;                             //address of first byte in readMemory
  uint8_t* actualAddress;


  if(passedData == 0)                                                               //if CNDTR is circle (because we want to read tab[closing] if we are at tab[0] )
  {
    actualAddress = startAddress + userValueCNDTR - 1;                              //address of the last read data
  }
  else
  {
    actualAddress = startAddress + passedData - 1;                                  //address of the last read data
  }

  return actualAddress;
}

static int Stop_UART3()
{
  int ret;

  //***************************Clear_UART_Registers***********************
  USART3->BRR = 0;                                                                   //clear baud rate register
  USART3->CR1 = 0;                                                                   //clear control register 1
  USART3->CR3 = 0;                                                                   //clear control register 3
  USART3->SR |= USART_SR_TC;                                                  	     //clear interrupt transmission complete

  //***************************STOP_Clock_DMA_UART************************
  RCC->APB1ENR &=~RCC_APB1ENR_USART3EN;                                              //clear clock for USART3
  RCC->AHB1ENR &=~RCC_AHB1ENR_DMA1EN;                                                //clear clock for DMA1

  is_init = 0;                                                                       //Clear UART init flag
  ret = E_UART_OK;

  return ret;
}


//****************************************APPLICATION INTERFACE******************************

int UART_Init(UARTcfg* config)
{
  int ret;
  int statusClock;
  int statusDMA;
  int statusUART;

  switch(config->uart)
  {
    case UART_1:
    case UART_2:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    case UART_3:
    {
      ApplicationHandler3 = config->UARTHandler;                                                    //application handler
      statusClock = Config_Clock_UART3();                                                           //config clock for DMA and UART
      statusDMA   = Config_DMA_UART3();                                                             //config DMA for UART_RX and UART_TX
      statusUART  = Config_UART3(config);                                                           //config UART
      break;
    }
    case UART_4:
    case UART_5:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    default:
      ret = E_UART_INVALID_PARAM;
  }

  if(statusClock != 0 || statusDMA != 0 || statusUART != 0)                                         //check status of peripherals config
  {
    ret = E_UART_NOT_INITIALIZED;
  }
  else
  {
    ret = E_UART_OK;
    is_init = 1;
  }

  return ret;
}

int UART_Send(uint8_t *data_send,uint32_t send_length, UARTcfg* config)
{
  int ret;

  if(is_init == 1)
  {
    switch(config->uart)
    {
      case UART_1:
      {
        ret = E_UART_NOT_IMPLEMENTED;
        break;
      }
      case UART_2:
      case UART_3:
      {
        ret = Send_UART3(data_send, send_length, config);                                             //send data by UART1
        break;
      }
      case UART_4:
      case UART_5:
      {
        ret = E_UART_NOT_IMPLEMENTED;
        break;
      }
      default:
      {
        ret = E_UART_INVALID_PARAM;
        break;
      }
    }
  }
  else
  {
    ret = E_UART_NOT_INITIALIZED;
  }

return ret;
}

int UART_Read(UARTcfg* config)
{
  int ret;

  if(is_init == 1)                                                                  //check if UART init
  {
    switch(config->uart)
    {
    case UART_1:
    {
      ret = E_UART_NOT_IMPLEMENTED;
     break;
    }
    case UART_2:
    case UART_3:
    {
      ret = Read_UART3(config);
      break;
    }
    case UART_4:
    case UART_5:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    default:
      ret = E_UART_INVALID_PARAM;
      break;
    }
  }
  else
  {
    ret = E_UART_NOT_INITIALIZED;
  }

return ret;
}

uint8_t* UART_Pointner(UARTcfg* config)
{
  uint8_t* pointner;

  switch(config->uart)
  {
    case UART_1:
    {
      break;
    }
    case UART_2:
    case UART_3:
    {
      pointner = Pointner_UART3(config);
      break;
    }
    case UART_4:
    case UART_5:
    {
      break;
    }
    default:
    {
      break;
    }
  }

  return pointner;
}

int NVIC_UART_IRQEnable(UARTcfg* config)
{
  int ret;

  switch(config->uart)
  {
    case UART_1:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    case UART_2:
    case UART_3:
    {
      //****************************interrupt_form_DMA_RX*******************************
      DMA1_Stream1->CR |= DMA_SxCR_TCIE;                                                //permission for interrupt after complete transmission (memory is full)
      NVIC_EnableIRQ(DMA1_Stream1_IRQn);                                                //permission for interrupt from DMA

      //***************************interrupt_form_DMA_TX*******************************
      DMA1_Stream3->CR |= DMA_SxCR_TCIE; ;                                               //permission for interrupt after complete transmission
      NVIC_EnableIRQ(DMA1_Stream3_IRQn);                                                 //permission for interrupt from DMA

      ret = E_UART_OK;
      break;
    }
    case UART_4:
    case UART_5:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    default:
    {
      ret = E_UART_INVALID_PARAM;
      break;
    }
  }

  return ret;
}

int NVIC_UART_IRQDisable(UARTcfg* config)
{
  int ret;

  switch(config->uart)
  {
    case UART_1:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    case UART_2:
    case UART_3:
    {
      //****************************interrupt_form_DMA_RX*******************************
      DMA2_Stream1->CR &= ~DMA_SxCR_TCIE;                                                //disable permission for interrupt after complete transmission (memory is full)

      //***************************interrupt_form_DMA_TX*******************************
      DMA2_Stream3->CR &= ~DMA_SxCR_TCIE;                                                //disable permission for interrupt after complete transmission

      ret = E_UART_OK;
      break;
    }
    case UART_4:
    case UART_5:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    default:
    {
      ret = E_UART_INVALID_PARAM;
      break;
    }
  }

  return ret;
}

uint32_t UART_CountByte(UARTcfg* config)
{


  return 0;
}

int UART_Stop(UARTcfg* config)
{
  int ret;

  switch(config->uart)
  {
    case UART_1:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    case UART_2:
    case UART_3:
    {
      ret = Stop_UART3();
      break;
    }
    case UART_4:
    case UART_5:
    {
      ret = E_UART_NOT_IMPLEMENTED;
      break;
    }
    default:
    {
      ret = E_UART_INVALID_PARAM;
      break;
    }
  }

  return ret;
}


