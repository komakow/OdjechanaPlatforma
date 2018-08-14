/**
 *@motorsControl.h
 *
 *@Created on: 7.06.2018
 *@Author: KamilM
 *
 *@brief  Header code of function used to control motors
 */
#ifndef HEADER_MOTORSCONTROL_H_
#define HEADER_MOTORSCONTROL_H_



/**
 * @brief Function used to run car forward with specific speed
 *
 * @param uint8_t speedLeft - speed of left wheel
 * @param uint8_t speedRight - speed of left wheel
 *
 * @return Status of operation
 */
int RunForward(uint8_t speedLeft, uint8_t speedRight);

/**
 * @brief Function used to run car reverse with specific speed
 *
 * @param uint8_t speedLeft - speed of left wheel
 * @param uint8_t speedRight - speed of left wheel
 *
 * @return Status of operation
 */
int RunReverse(uint8_t speedLeft, uint8_t speedRight);


#endif /* HEADER_MOTORSCONTROL_H_ */
