/*
 *@encoder.c
 *
 *@Created on: 12.04.2018
 *@Author: KamilM
 *
 *@brief Driver for magnetic encoder for Pololu Romi platform.
 */

#ifndef ENCODER_H_
#define ENCODER_H_

typedef int err;

#define E_OK					       0			//Operation success
#define E_NOT_INITIALIZED		-1			//Encoder not initialize
#define E_NOT_READ				  -2			//Data is not read from register
//...TODO multiple if necessary

/*
 * @brief Initial function for encoder
 *
 * @param None
 *
 * @return Status of operation
 */
err Encoder_Init();

/*
 * @brief Function to speed calculation from encoder and send by UART
 *
 * @param None
 *
 * @return Speed in [cm/s] or error
 */
float Encoder_SpeedSend();

/*
 * @brief Function enabling interrupts used by Encoder driver
 *
 * @param None
 *
 * @return Status of operation
 */
err Encoder_IRQ_Enable();

/*
 * @brief Function disabling interrupts used by Encoder driver
 *
 * @param None
 *
 * @return Status of operation
 */
err Encoder_IRQ_Disable();

/*
 * @brief Function for deinitialization encoder and all connected peripheral
 *
 * @param None
 *
 * @return Status of operation
 */
err Encoder_DeInit();


#endif /* ENCODER_H_ */
