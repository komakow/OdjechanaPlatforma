/**
 *@Algorythm.h
 *
 *@Created on: 30.05.2018
 *@Author: KamilM
 *
 *@brief Header code of function used to control main algorithm of 'odjechana platforma'
 */

#ifndef HEADER_ALGORYTHM_H_
#define HEADER_ALGORYTHM_H_

/**
 * @brief Function init all of used periphery
 *
 * @param None
 *
 * @return None
 */
int PeripheryInit();

/**
 * @brief Function used when Bluetooth is disconnect. This function stops all of motors and other periphery
 * i.e used during normal work
 *
 * @param None
 *
 * @return None
 */
void BTDisconnected();

/**
 * @brief Delay constant pieces of time
 *
 * @param None
 *
 * @return None
 */
void delay();

/**
 * @brief Function contains algorithm used when value of speed is send from UART.
 * i.e That function can control wheel, run car forward or reverse.
 *
 * @param None
 *
 * @return Status of operation
 */
int RunCar();

/**
 * @brief Function used to measure and send by UART all of sonic sensor
 *
 * @param None
 *
 * @return Status of operation
 */
uint8_t DistanceReadSend(uint8_t* distance);

/**
 * @brief Function used to reset all of used drivers like encoder etc.
 *
 * @param None
 *
 * @return Status of operation
 */
uint8_t ResetPheriph();


#endif /* HEADER_ALGORYTHM_H_ */
