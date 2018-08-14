/**
 *@motorsControl.c
 *
 *@Created on: 7.06.2018
 *@Author: KamilM
 *
 *@brief Source code of function used to control motors. This file is a upper layer of Hbridge.h
 */
#include "main.h"


int RunForward(uint8_t speedLeft, uint8_t speedRight)
{
  int ret = 0;

  ret =  WheelControl(Left,Forward,speedLeft);
  ret += WheelControl(Right,Forward,speedRight);

  return ret;
}

int RunReverse(uint8_t speedLeft, uint8_t speedRight)
{
  int ret = 0;

  ret =  WheelControl(Left,Reverse,speedLeft);
  ret += WheelControl(Right,Reverse,speedRight);

  return ret;
}
