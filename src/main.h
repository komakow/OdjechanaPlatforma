/**
 *@main.h
 *
 *@Created on: 30.05.2018
 *@Author: KamilM
 *
 *@brief Header file of main file. This file include all of header file used in project and few of variable
 */

#ifndef MAIN_H_
#define MAIN_H_

//*****************************Include*******************************
#include "stm32f4xx.h"
#include "defGPIO.h"
#include "encoder.h"
#include "GPIODriver.h"
#include "Hbridge.h"
#include "HC_SR04.h"
#include "uart.h"
#include "Algorythm.h"
#include "motorsControl.h"

//***************************Variables********************************

UARTcfg UART;                                                                       //uart config scructure


#endif /* MAIN_H_ */
