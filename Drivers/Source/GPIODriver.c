/**
 * @file DriverGPIO.c
 *
 * @brief Source code of abstract controller of GPIO working at STM32L476VGT.
 *
 * @Created on: 12.11.2017
 * @Author: malskik
 *
 */
//*********************************************INCLUDE**********************************

#include "GPIODriver.h"
#include "stm32f4xx.h"


//***********************************STATIC FUNCTION PROTOTYPE************************

static void GPIOClock(GPIOcfgType* gpio_cfg);
static void GPIOOTYPER(GPIOcfgType* gpio_cfg, GPIO_TypeDef* gpio, uint32_t** reset_maskptr, uint32_t** pin_numberptr);
static void GPIOMODER(GPIOcfgType* gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr);
static void GPIOOSPEEDR(GPIOcfgType* gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr);
static void GPIOPUPDR(GPIOcfgType* gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr);
static void GPIOALTER(GPIOcfgType* gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr);
static void GPIOPORT(GPIOPortType port, GPIO_TypeDef** gpio);
static int  GPIOCheck(GPIOcfgType* gpio_cfg);
static uint32_t GPIOIsOutput( GPIO_TypeDef* gpio, GPIOPinType pin );
static uint32_t GPIOIsAlternate(GPIO_TypeDef* gpio, GPIOPinType pin);


//****************************************STATIC FUNCTION******************************

static void GPIOClock(GPIOcfgType *gpio_cfg)
{
   uint32_t pClock;
   uint32_t error = 0;

   switch (gpio_cfg->port)                                                       //assigned clock for GPIO using CMSIS
   {
   case PORTA:
     pClock=RCC_AHB1ENR_GPIOAEN;
     break;
   case PORTB:
     pClock=RCC_AHB1ENR_GPIOBEN;
     break;
   case PORTC:
     pClock=RCC_AHB1ENR_GPIOCEN;
     break;
   case PORTD:
     pClock=RCC_AHB1ENR_GPIODEN;
     break;
   case PORTE:
     pClock=RCC_AHB1ENR_GPIOEEN;
     break;
   case PORTF:
     pClock=RCC_AHB1ENR_GPIOFEN;
     break;
   case PORTG:
     pClock=RCC_AHB1ENR_GPIOGEN;
     break;
   case PORTH:
     pClock=RCC_AHB1ENR_GPIOHEN;
     break;

   default:
     error = 1;                                                                   //in case if gpio_cfg->port is incorrect set to 0, to do not change anything in RCC

   }

  //*******************set CLOCK register**************
   if(error == 0)
   {
     uint32_t temp = (RCC->AHB1ENR ) & pClock;
     if(!(temp & pClock ))                                                        //if clock is not set yet
     {
       RCC->AHB1ENR |= pClock;
     }
   }

}

static void GPIOOTYPER(GPIOcfgType *gpio_cfg, GPIO_TypeDef* gpio, uint32_t** reset_maskptr, uint32_t** pin_numberptr)
{

  uint32_t reset_mask;                                                           //used to clear specific pin place in registers
  uint32_t pin_number;                                                           //specific pin places in 32bit register

  //*******************set OTYPER register*************
  if (gpio_cfg->typ == openDrain)
  {
    gpio->OTYPER |= gpio_cfg->pin;                                              //one pin has 1 config bit in this register
  }
  else if (gpio_cfg->typ == pushPull)
  {
    gpio->OTYPER &= ~gpio_cfg->pin;
  }

  pin_number = __builtin_ctz(gpio_cfg->pin) * 2;                               //count trailing zeros (how many zeros form the right). One pin has 2 config bits in this registers
  *pin_numberptr = &pin_number;                                                //address of pin_number variable
  reset_mask = ~(0x03 << pin_number);                                          //used to reset specific bits in the register
  *reset_maskptr = &reset_mask;                                                //address of reset_mask variable

}

static void GPIOMODER(GPIOcfgType *gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr)
{

  uint32_t reg_val;                                                             //used to write to the register
  uint32_t reset_mask = *reset_maskptr;                                         //used to clear specific pin place in registers
  uint32_t pin_number = *pin_numberptr;                                         //specific pin places in 32bit register

  //*****************set MODER register****************
  reg_val = gpio->MODER;                                                        //read actual value from register
  reg_val &= reset_mask;                                                        //reset bits
  reg_val |= gpio_cfg->mode << pin_number;                                      //shift bits to specific place
  gpio->MODER = reg_val;                                                        //write to register

}

static void GPIOOSPEEDR(GPIOcfgType *gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr)
{

  uint32_t reg_val;                                                              //used to write to the register
  uint32_t reset_mask = *reset_maskptr;                                          //used to clear specific pin place in registers
  uint32_t pin_number = *pin_numberptr;                                          //specific pin places in 32bit register

  //*****************set OSPEEDR register**************
  if (gpio_cfg->speed <= fast)
  {
    reg_val = gpio->OSPEEDR;                                                      //read actual value from register
    reg_val &= reset_mask;                                                        //reset bits
    reg_val |= gpio_cfg->speed << pin_number;                                     //shift bits to specific place
    gpio->OSPEEDR = reg_val;                                                      //write to register
  }
}

static void GPIOPUPDR(GPIOcfgType *gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr)
{
  uint32_t reg_val;                                                              //used to write to the register
  uint32_t reset_mask = *reset_maskptr;                                          //used to clear specific pin place in registers
  uint32_t pin_number = *pin_numberptr;                                          //specific pin places in 32bit register

  //*****************set PUPDR register****************
  if (gpio_cfg->pull <= 2)
  {
    reg_val = gpio->PUPDR;                                                        //read actual value from register
    reg_val &= reset_mask;                                                        //reset bits
    reg_val |= gpio_cfg->pull << pin_number;                                      //shift bits to specific place
    gpio->PUPDR = reg_val;                                                        //write to register
  }
}

static void GPIOALTER(GPIOcfgType *gpio_cfg, GPIO_TypeDef* gpio, uint32_t* reset_maskptr, uint32_t* pin_numberptr)
{
  uint32_t reg_val;                                                              //used to write to the register
  uint32_t reset_mask = *reset_maskptr;                                          //used to clear specific pin place in registers
  uint32_t pin_number = *pin_numberptr;                                          //specific pin places in 32bit register

  //*****************set ALTER register****************
  if (gpio_cfg->mode == alternate)
  {
    if (gpio_cfg->alter < 16)
    {
      pin_number *= 2;                                                            //one pin has 4 config bits
      reset_mask = ~(0b1111 << pin_number);
      if (pin_number > 28)                                                        //config MSB of AFR register (in stm32l4 are two alter registers)
      {
        pin_number -= 32;                                                        //shift data to beginning of second register
        reg_val = gpio->AFR[1];                                                 //read actual value from AFR[1] register
        reg_val &= reset_mask;
        reg_val |= gpio_cfg->alter << pin_number;                                 //shift data to specific place in the register
        gpio->AFR[1] = reg_val;
      }
      else
      {
        reg_val = gpio->AFR[0];                                                   //config LSB of AFR register
        reg_val &= reset_mask;
        reg_val |= gpio_cfg->alter << pin_number;
        gpio->AFR[0] = reg_val;
      }
    }
  }

}

static void GPIOPORT(GPIOPortType port, GPIO_TypeDef** gpio)
{

  switch (port)                                                           //assigned GPIO to PORT using CMSIS
  {
    case PORTA :
      *gpio = GPIOA;
      break;
    case PORTB :
      *gpio = GPIOB;
      break;
    case PORTC :
      *gpio = GPIOC;
      break;
    case PORTD :
      *gpio = GPIOD;
      break;
    case PORTE :
      *gpio = GPIOE;
      break;
    case PORTF :
      *gpio = GPIOF;
      break;
    case PORTG :
      *gpio = GPIOG;
      break;
    case PORTH :
      *gpio = GPIOH;
      break;
  }

}

static int GPIOCheck(GPIOcfgType* gpio_cfg)
{

  int ret = E_GPIO_OK;

  //******************************check correctness of gpio_cfg->pin user value******************
  int retPIN = -1;
  if (gpio_cfg->pin == 0)
  {
    retPIN = 0;
  }
  int b = 1;
  do
  {
    if (gpio_cfg->pin == b)
    {
      retPIN = b;
    }
    b = b << 1;
  }
  while (b <= PIN15);

  //******************************check correctness of gpio_cfg->mode user value******************
  int retMode = -1;
  for (int a = 0; a <= analog; a++)
  {
    if (gpio_cfg->mode == a)
    {
      retMode = a;
    }
  }

  //******************************check correctness of gpio_cfg->typ user value******************
  int retTyp = -1;
  for (int a = 0; a <= openDrain; a++)
  {
    if (gpio_cfg->typ == a)
    {
      retTyp = a;
    }
  }

  //******************************check correctness of gpio_cfg->speed user value******************
  int retSpeed = -1;
  for (int a = 0; a <= fast; a++)
  {
    if (gpio_cfg->speed == a)
    {
      retSpeed = a;
    }
  }

  //******************************check correctness of gpio_cfg->pull user value******************
  int retPull = -1;
  for (int a = 0; a <= pullDown; a++)
  {
    if (gpio_cfg->pull == a)
    {
      retPull = a;
    }
  }

  //******************************check correctness of gpio_cfg->alter user value******************
  int retAlter = -1;
  if (gpio_cfg->mode == alternate)
    for (int a = 0; a <= AF15; a++)
    {
      if (gpio_cfg->alter == a)
      {
        retAlter = a;
      }
    }

  //******************************check correctness of gpio_cfg->port user value******************
  int retPort = -1;
  for (int a = 1; a <= PORTH; a++)
  {
    if (gpio_cfg->port == a)
    {
      retPort = a;
    }
  }

  //****************************check if is any error*********************************************
  if ((gpio_cfg->mode == alternate) && (retAlter == -1))
  {
    ret = E_GPIO_INVALID_PARAM;
  }
  else if ((retPIN != -1) && (retMode != -1) && (retTyp != -1) && (retSpeed != -1) && (retPull != -1)
           && (retPort != -1))
  {
    ret = E_GPIO_OK;
  }
  else
  {
    ret = E_GPIO_INVALID_PARAM;
  }

  return ret;
}

static uint32_t GPIOIsOutput( GPIO_TypeDef* gpio, GPIOPinType pin )
{
  uint32_t ret;                                                                  //value to return
  uint32_t pinNumber;                                                            //specific pin places in 32bit register
  uint32_t outputMode;

  pinNumber = __builtin_ctz(pin) * 2;                                            //count trailing zeros (how many zeros form the right). One pin has 2 config bits in this registers
  outputMode = 0x01 << pinNumber;                                                //used to term place at moder register specific pin

  if (gpio->MODER & outputMode)                                                  //check if specific pin is at output mode
  {
    ret = 1;                                                                     //if is output return 1
  }
  else
  {
    ret = 0;                                                                     //if is not output return 0
  }

  return ret;
}

static uint32_t GPIOIsAlternate(GPIO_TypeDef* gpio, GPIOPinType pin)
{
  uint32_t ret;                                                                  //value to return
  uint32_t pinNumber;                                                            //specific pin places in 32bit register
  uint32_t alternateMode;

  pinNumber = __builtin_ctz(pin) * 2;                                            //count trailing zeros (how many zeros form the right). One pin has 2 config bits in this registers
  alternateMode = 0x02 << pinNumber;                                             //used to term place at moder register specific pin

  if (gpio->MODER & alternateMode)                                               //check if specific pin is at alternate mode
  {
    ret = 1;                                                                     //if is an alternate return 1
  }
  else
  {
    ret = 0;                                                                     //if is not an alternate return 0
  }

  return ret;
}


//***************************************INTERFACE FUNCTION****************************

int gpioCfg(GPIOcfgType* gpio_cfg)
{
	/*
	  gpio - GPIOx
	  mode - MODER
	  typ - OTYPER
	  speed - OSPEEDR
	  pull - PUPDR
	  alter - ALF

	 * MODER:
	  00: Input mode
		01: General purpose output mode
		10: Alternate function mode
		11: Analog mode (reset state)

	 *OTYPER
	 	0: Output push-pull (reset state)
		1: Output open-drain

	 *OSPEEDR
	 	00: Low speed
		01: Medium speed
		10: High speed
		11: Very high speed

	 *PUPDR
		00: No pull-up, no pull-down
		01: Pull-up
		10: Pull-down
		11: Reserved
	 */

  int ret = E_GPIO_OK;

  GPIO_TypeDef* gpio;                                                                //using to implemented CMISIS in driver

  uint32_t* reset_maskptr = 0;                                                       //used to clear specific pin place in registers
  uint32_t* pin_numberptr = 0;                                                       //specific pin places in 32bit register

  if (GPIOCheck(gpio_cfg) == E_GPIO_OK)                                              //check correctness of fields value in structure GPIOcfgType
  {
    GPIOPORT(gpio_cfg->port, &gpio);                                                 //implemented CMISIS macros
    GPIOClock(gpio_cfg);                                                             //config clock for specific GPIO
    GPIOOTYPER(gpio_cfg, gpio, &reset_maskptr, &pin_numberptr);                      //config OTYPER register and modify reset_mask and pin_number variables
    GPIOMODER(gpio_cfg, gpio, reset_maskptr, pin_numberptr);                         //config MODER register
    GPIOOSPEEDR(gpio_cfg, gpio, reset_maskptr, pin_numberptr);                       //config OSPEEDR register
    GPIOPUPDR(gpio_cfg, gpio, reset_maskptr, pin_numberptr);                         //config PUPDR register
    GPIOALTER(gpio_cfg, gpio, reset_maskptr, pin_numberptr);                         //config ALTER register

    if(gpio_cfg->mode == output)                                                     //if gpio_cfg->mode is set as output, set default value of specific pin is "CLEAR"
    {
      gpioPinSetState(gpio_cfg->port, gpio_cfg->pin, clear);                         //set specific pin to "0"
    }

  }
  else
  {
    ret = E_GPIO_INVALID_PARAM;
  }

  return ret;
}


int gpioPinSetState(GPIOPortType port, GPIOPinType pin, GPIOSetType state)
{
  int ret = E_GPIO_OK;
  GPIO_TypeDef* gpio = 0;
  GPIOPORT(port, &gpio);                                                      //implemented CMISIS macros

  if (GPIOIsOutput(gpio, pin) == 1)                                           //check if specific pin is output
  {

    if (state == clear)                                                       //reset pin
    {
      gpio->BSRRH = pin;
    }

    else if (state == set)	                                                   //set pin
    {
      gpio->BSRRL = pin;
    }
    else
    {
      ret = E_GPIO_NOT_INITIALIZE;
    }

  }
  else
  {
    ret = E_GPIO_NOT_OUTPUT_MODE;
  }

  return ret;
}


int gpioGetPinState(GPIOPortType port, GPIOPinType pin)
{
  int ret;
  GPIO_TypeDef* gpio = 0;
  GPIOPORT(port, &gpio);                                                //implemented CMISIS macros

  if (GPIOIsAlternate(gpio, pin) == 0)                                  //check if specific pin is not at alternate mode
  {
    if (gpio->IDR & pin)                                                //if pin is set
    {
      ret = set;
    }
    else if (~(gpio->IDR & pin))                                        //if pin is reset
    {
      ret = clear;
    }

  }
  else
  {
    ret = E_GPIO_ALTERNATE_MODE;
  }

  return ret;
}

