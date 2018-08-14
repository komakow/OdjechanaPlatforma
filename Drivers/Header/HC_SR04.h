/*
 *@HC-SR04.h
 *
 *@Created on: 07.05.2018
 *@Author: Kamil Malski
 *
 *@brief Header file of ultrasonic sensor used by "Odjechana Platforma" project.
 */

#ifndef HC_SR04_H_
#define HC_SR04_H_

/*
 * @brief Typedef of sensor installed on platform
 * TODO: Multiply if necessary
 */
typedef enum{
	HC_left		=	0x00,
	HC_center	=	0x01,
	HC_right	=	0x02,
	HC_back		=	0x03

}HCSRTypeDef;

/*
 * @brief Function used to initial all of sonic sensor installed on platform
 *
 * @param None
 *
 * @return None
 */
void HC_SR04_Init();

/*
 * @brief Function used to read distance from sonic sensor installed on platform
 *
 * @param - HCSRTypeDef sensor - number of sensor installed on platform
 *
 * @return Read distance from sensor in [cm]
 */
float HC_SR04_ReadDistance(HCSRTypeDef sensor);

/*
 * @brief Function used to deinitialization all of periphery used by sonic sensor
 *
 * @param None
 *
 * @return None
 */
void HC_SR04_DeInit();


#endif /* HC_SR04_H_ */
