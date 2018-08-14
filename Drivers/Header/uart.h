/*
 * @file uart.h
 * @brief interface of UART controller used to receive and transmit data using DMA.
 * ie. Word length = 8 bit data + 1 stop bit or 9 bit data ( 1 parity control bit) + 1 stop bit
 * ie. Baundrate = 9600
 * ie. UART_RX = GPIOB.7
 * ie. UART_TX = GPIOB.6
 * ie. Clock = 48 Mhz (do not change!)
 *
 *  Created on: 17.05.2017
 *      Author: malskik
 *
 */


#ifndef UART_H_
#define UART_H_


/*@brief Error representation
 *
 */
#define E_UART_OK                	  0          //Operation successful
#define E_UART_NOT_INITIALIZED       -1          //UART not initializated
#define E_UART_NOT_IMPLEMENTED       -2          //UART not implemented on specific hardware
#define E_UART_INVALID_PARAM    -3          //Invalid input parameter
#define E_UART_NOT_SEND              -4          //Data not send
#define E_UART_NOT_READ              -5          //Data not read


/*
 * @brief Parity Control TypeDef
 */
typedef enum
{
    unused = 0,
    used   = 1
} ParityType;

/*
 * @brief Numeric representation of abstract UART
 */
typedef enum
{
  UART_1 = 0,
  UART_2 = 1,
  UART_3 = 2,
  UART_4 = 3,
  UART_5 = 4
} UARTType;


/**
 * @brief UART read data configuration structure
 */
typedef struct
{
    void          (*UARTHandler)(void);               //handler to UART function
    ParityType    parityControl;                      //parity control for UART
    UARTType      uart;                               //numeric representation of abstract UART
    uint32_t      readLength;                         //length of place in memory to save data from UART
    int           baundRate;                          //baund rate for UART
    uint8_t*      readMemory;                         //pointner to place in memory where read memory should be saved

} UARTcfg;


/* @brief Function initializes UART
 *
 * @returns Status of initialization
 */
int UART_Init(UARTcfg* config);

/*
 * @brief Function send data using DMA
 *
 * @param data_send - pointner to send data
 * @param send_length - length of send data
 *
 * @return Status of transmit
 */
int UART_Send(uint8_t *data_send,uint32_t send_length, UARTcfg* config);

/*
 * @brief Function initializes UART receiver by param from UARTcfg struct
 * ie. Function used when parameters of UARTcfg changed
 *
 * @param UARTcfg - struct with initializes parameter
 *
 * @return Status of data read
 */
int UART_Read(UARTcfg* config);

/*
 * @brief Function used to generate address of the last read data from UART_RX
 *
 * @param UARTcfg - struct with initializes parameter
 *
 * @return Address of the last read data from UART_RX
 */
uint8_t* UART_Pointner(UARTcfg* config);

/*
 * @brief Function enable interrupt from UART DMA
 *
 * @param UARTcfg - struct with initializes parameter
 *
 * @return Status of IRQ enable
 */
int NVIC_UART_IRQEnable(UARTcfg* config);

/*
 * @brief Function disable interrupt from UART DMA
 *
 * @param UARTcfg - struct with initializes parameter
 *
 * @return Status of IRQ disable
 */
int NVIC_UART_IRQDisable(UARTcfg* config);

/*
 * @brief Function used to count read byte
 *
 * @param UARTcfg - struct with initializes parameter
 *
 * @return Number of read bytes since the last deletion (default since the first read byte)
 */
uint32_t UART_CountByte(UARTcfg* config);

/*
 * @brief Function stop UART and DMA2 and clears all of their registers
 *
 * @return status of UART stop
 */
int UART_Stop(UARTcfg* config);

#endif /* UART_H_ */
