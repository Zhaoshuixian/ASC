
#ifndef _COMM_H_
#define _COMM_H_

#include "stm32l4xx_hal.h"
#include "stdio.h"

#define DEBUG_MODE

/*! Initializes the I2C communication peripheral. */
unsigned char I2C_Init(unsigned long clockFreq);

/*! Writes data to a slave device. */
unsigned char I2C_Write(unsigned char slaveAddress,
                        unsigned char* dataBuffer,
                        unsigned char bytesNumber,
                        unsigned char stopBit);

/*! Reads data from a slave device. */
unsigned char I2C_Read(unsigned char slaveAddress,
                       unsigned char* dataBuffer,
                       unsigned char bytesNumber,
                       unsigned char stopBit);

/*! Initializes the SPI communication peripheral. */
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg);

/*! Initializes the SPI communication peripheral. */
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg);

/*! Reads data from SPI. */
unsigned char SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber);

/*! Writes data to SPI. */
unsigned char SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber);

void Error_Handler(void);                        

#endif // _COMMUNICATION_H_
