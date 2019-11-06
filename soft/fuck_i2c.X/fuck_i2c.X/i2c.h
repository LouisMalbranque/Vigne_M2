/*
 * File:   i2c.h
 * Author: lal
 *
 * Created on 23 mars 2015, 13:53
 */

#ifndef _I2C_H
#define	_I2C_H

#include "general.h"

/*Mapping table concerning the Serial Pins for PIX18F46K22 uC.
 * SCK (SPI) and SCL (I2C) can be wired to D0 or C3
 * SDI (SPI's MISO) and SDA can be wired to D1 or C4
 * SDO (SPI's MISO) can be wired to D4 or C5
 * SS (SPI's Slave Select) can be wired to A5 or D3 */

/*Pins arbitrary choosen : */
#define I2C_SCL1_DIR         TRISCbits.TRISC3        // Direction bit for I2C clock (SCL1)
#define I2C_SDA1_DIR         TRISCbits.TRISC4        // Direction bit for I2C serial data (SDA1)

/*It means that when creating the SPI serial interface : 
 SCK MUST be set to D0 (aka SCK2)
 SDI MUST be set to D1 (aka SDI2)
 SS MUST be set to D3 (aka SS2)
 SDO MUST be set to D4 (aka SDO2)*/

/*Address of the choosen humidity_temperature sensor*/
#define HIH6031_ADDRESS 0x27

#define I2C_READ            READ                    // I2C read mode
#define I2C_WRITE           WRITE                   // I2C write mode

void i2c_init(void); //Initialize the right registers. Clear the necessary bits in order for the uC to be prepared to use I2C.
void i2c_waitForIdle(void); //Check and maintain the Master in an idle state
void i2c_start(void); //Initiate the start conditions on SDA1/SCL1 pins
void i2c_repStart(void); //Initiate the repeated start condition
void i2c_stop(void); //initiate the stop condition
UINT8_T i2c_read(void); //Read data from the SSP1BUF register
void i2c_write(UINT8_T data); //Write to the SSP1BUF register
void i2c_ACK(void); //Send an ACK to the Slave
void i2c_NAK(void); //Send an NAK to the Slave

#endif /* _I2C_H */

