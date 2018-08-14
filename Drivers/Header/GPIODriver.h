/**
 * @file	DriverGPIO.h
 * @brief Interface of GPIO used to configuration ports and pins and check the status of pins
 *
 * @Created on: 12.11.2017
 * @Author: KM
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_


/**
 * @brief Numeric representation of error
 */
#define E_GPIO_OK                           0                   //Operation successful
#define E_GPIO_INVALID_PARAM               -1                   //Invalid parameter send to function
#define E_GPIO_NOT_INITIALIZE              -2                   //Periphery is not initialize
#define E_GPIO_NOT_OUTPUT_MODE             -3                   //Pin not config as a output
#define E_GPIO_ALTERNATE_MODE              -4                   //Pin is config as an alternate


/**
 * @brief numeric representation state of pins
 *
 */
typedef enum
{
  clear = 0x0,
  set   = 0x1

}GPIOSetType;

/**
 * @brief numeric representation mode of GPIO pins
 *
 */
typedef enum
{
  input     = 0x0,
  output    = 0x1,
  alternate = 0x2,
  analog    = 0x3

}GPIOModeType;

/**
 * @brief numeric representation of pins
 *
 */
typedef enum
{
  PIN0  = 0x00000001,
  PIN1  = 0x00000002,
  PIN2  = 0x00000004,
  PIN3  = 0x00000008,
  PIN4  = 0x00000010,
  PIN5  = 0x00000020,
  PIN6  = 0x00000040,
  PIN7  = 0x00000080,
  PIN8  = 0x00000100,
  PIN9  = 0x00000200,
  PIN10 = 0x00000400,
  PIN11 = 0x00000800,
  PIN12 = 0x00001000,
  PIN13 = 0x00002000,
  PIN14 = 0x00004000,
  PIN15 = 0x00008000

}GPIOPinType;

/**
 * @brief numeric representation output type of pins
 *
 */
typedef enum
{
  pushPull  = 0x0,
  openDrain = 0x1

}GPIOOutputType;

/**
 * @brief numeric representation speed of pins
 *
 */
typedef enum
{
  low    = 0x0,
  medium = 0x1,
  high   = 0x2,
  fast   = 0x3

}GPIOSpeedType;

/**
 * @brief numeric representation pull-up/pull-down of pins
 *
 */
typedef enum
{
  floating  = 0x0,
  pullUp    = 0x1,
  pullDown  = 0x2

}GPIOPullType;

/**
 * @brief numeric representation alternate function of pins
 */
typedef enum
{
  AF0   = 0x0000,
  AF1   = 0b0001,
  AF2   = 0b0010,
  AF3   = 0b0011,
  AF4   = 0b0100,
  AF5   = 0b0101,
  AF6   = 0b0110,
  AF7   = 0b0111,
  AF8   = 0b1000,
  AF9   = 0b1001,
  AF10  = 0b1010,
  AF11  = 0b1011,
  AF12  = 0b1100,
  AF13  = 0b1101,
  AF14  = 0b1110,
  AF15  = 0b1111

}GPIOAlterType;

/**
 * @brief abstract representation of GPIO port
 */
typedef enum
{
  PORTA = 0x01,
  PORTB = 0x02,
  PORTC = 0x03,
  PORTD = 0x04,
  PORTE = 0x05,
  PORTF = 0x06,
  PORTG = 0x07,
  PORTH = 0x08

}GPIOPortType;

typedef struct
{
    /**
     * PIN1
     * PIN2
     * ...
     */
    GPIOPinType pin;	                          	//number of pin


    /**
     *input     = 0,
     *output    = 0x1,
     *alternate = 0x2,
     *analog    = 0x3
     */
    GPIOModeType mode;                          	//GPIO mode register


    /**
     *pushPull   = 0x0,
     *openDrain  = 0x1
     */
    GPIOOutputType typ;                         	//GPIO port output type register


    /**
     *low    = 0x0,
     *medium = 0x1,
     *high   = 0x2,
     *fast   = 0x3
     */
    GPIOSpeedType speed;                        	//GPIO port output speed register


    /**
     *floating = 0,
     *pullUp   = 0x1,
     *pullDown = 0x2
     */
    GPIOPullType pull;	                          //GPIO port pull-up/pull-down register


    /**
     *AF0    = 0,
     *AF1    = 0b0001,
     *AF2    = 0b0010,
     *...
     */
    GPIOAlterType alter;	                        //GPIO alternate function registers

    /**
     * PORTA,
     * PORTB,
     * PORTC,
     * ...
     */
    GPIOPortType port;                            //GPIO specific PORT

}GPIOcfgType;


/**
 * @brief Function initializes the GPIO port and pin
 *
 * @param *gpio_cfg - pointer to configuration structure of GPIO pin
 *
 * @return Function return status of initializes
 */
int gpioCfg(GPIOcfgType *gpio_cfg);

/**
 * @brief Function set the GPIO pin
 *
 * @param GPIOPortType port - Port with should be modify
 * @param GPIOPinType pin   - Pin with should be modify
 * @param state             - state of pin to set
 *
 * @return Function return status of change state
 */
int gpioPinSetState(GPIOPortType port, GPIOPinType pin , GPIOSetType state);

/**
 * @brief Function check status of GPIO pin
 *
 * @param GPIOPortType port - Port with should be modify
 * @param GPIOPinType pin   - Pin with should be modify
 *
 * @return Function return current state of specific pin
 */
int gpioGetPinState(GPIOPortType port, GPIOPinType pin);


#endif /* GPIO_DRIVER_H_ */
