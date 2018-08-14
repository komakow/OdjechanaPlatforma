/*
 *@defGPIO.h
 *
 *@Created on: 12.04.2018
 *@Author: KamilM
 *
 *@brief Header file with define all of used GPIO pins
 */

#ifndef DEFGPIO_H_
#define DEFGPIO_H_

#include <GPIODriver.h>


int GPIO_InitializeAll();


//Encoder CH1, first encoder
#define EncoderCh1_port				  PORTA
#define EncoderCh1_pin				  PIN0
GPIOcfgType EncoderCh1;

//Encoder CH2, first encoder
#define EncoderCh2_port				  PORTA
#define EncoderCh2_pin				  PIN1
GPIOcfgType EncoderCh2;

//Greed Led
#define LedGreen_port				    PORTD
#define	LedGreen_pin				    PIN12
GPIOcfgType LedGreen;

//Wheel Left PWM Channel_1
#define WheelLeftPWM_port			  PORTB
#define WheelLeftPWM_pin			  PIN4
GPIOcfgType WheelLeftPWM;

//Wheel Right PWM Channel_2
#define WheelRightPWM_port			PORTB
#define WheelRightPWM_pin			  PIN5
GPIOcfgType WheelRightPWM;

//Wheel Left Phase
#define WheelLeftPH_port			  PORTD
#define WheelLeftPH_pin				  PIN0
GPIOcfgType WheelLeftPH;

//Wheel Right Phase
#define WheelRightPH_port			  PORTD
#define WheelRightPH_pin			  PIN1
GPIOcfgType WheelRightPH;

//Sonic ECHO universal
#define HC_ECHO_port				    PORTB
#define HC_ECHO_pin					    PIN8
GPIOcfgType HC_ECHO;

//Trigger of left sonic sensor
#define HCTriggerLeft_port			PORTE
#define HCTriggerLeft_pin			  PIN0
GPIOcfgType HCTriggerLeft;

//Trigger of center sonic sensor
#define HCTriggerCenter_port		PORTE
#define HCTriggerCenter_pin			PIN1
GPIOcfgType HCTriggerCenter;

//Trigger of right sonic sensor
#define HCTriggerRight_port			PORTE
#define HCTriggerRight_pin			PIN2
GPIOcfgType HCTriggerRight;

#define HCTriggerBack_port			PORTE
#define	HCTriggerBack_pin			  PIN3
GPIOcfgType HCTriggerBack;

//UART3 Text
#define UARTText_port				    PORTB
#define UARTText_pin				    PIN10
GPIOcfgType UARTText;

//UART3 Read
#define UARTRead_port				    PORTB
#define UARTRead_pin				    PIN11
GPIOcfgType UARTRead;

//BlueTooth status
#define BTStatus_port				    PORTA
#define BTStatus_pin				    PIN4
GPIOcfgType BTStatus;

//Red diode
#define DiodeRed_port           PORTD
#define DiodeRed_pin            PIN14
GPIOcfgType DiodeRed;

//Encoder CH1, second encoder
#define EncoderCh3_port         PORTB
#define EncoderCh3_pin          PIN6
GPIOcfgType EncoderCh3;

//Encoder CH2, second encoder
#define EncoderCh4_port         PORTB
#define EncoderCh4_pin          PIN7
GPIOcfgType EncoderCh3;

//Orange diode
//Red diode
#define DiodeOrange_port        PORTD
#define DiodeOrange_pin         PIN13
GPIOcfgType DiodeOrange;
#endif /* DEFGPIO_H_ */
