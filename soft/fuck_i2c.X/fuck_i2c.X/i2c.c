/*
 * File:   i2c.c
 * Author: lal
 *
 * Created on 24 mars 2015, 08:09
 */

//#include <stdio.h>
//#include <stdlib.h>
#include <xc.h>
#include "general.h"
#include "i2c.h"

/*SSPxCONx are important registers : 
 -SSP1CON1 is used when initializing SCL1-SDA1 or SCK1/SDO1/SDI1 communications
 -SSP1CON2 is used when communicating with I2C */

void i2c_init(void) {
/*For the PIC18F46K22, two MSSP does exist, as two serial interfaces exists.
 MSSP1 has been choosen to stand for the I2C communication.
 For description, useful information can be found from the page 215 of the datasheet.*/
    
    I2C_SCL1_DIR = INP;                  // I2C clock and serial bits
    I2C_SDA1_DIR = INP;                  // SLC on RC3, SDA on RC4
    ANSELCbits.ANSC3 = DISABLE; //Disable analog input on C3
    ANSELCbits.ANSC4 = DISABLE; //Disable analog input on C4

    /*We have choosen I2C interface to be SCL1 and SDA, meaning that we'll choose the SSP1CON1 register to initialize.*/
    /*Page 235 of the datasheet tells how to put the I2C Master Mode.*/
    /*SSP1CON1 is one of the 2 control registers for MSSP1 */ 
    SSP1CON1bits.WCOL = 0; // no collision
    SSP1CON1bits.SSPOV1 = 0; // no overflow (allow ACK from slave)
    SSP1CON1bits.SSPEN1 = 1; // SCL1 and SDA1 as serial ports
    SSP1CON1bits.CKP = 0;//Unused in I2C Master mode
    SSP1CON1bits.SSPM = 0b1000;//Enable I2C Master mode, with Fclk = Fosc/(4*SSP1ADD+1))
    Nop();                           
    Nop();
    Nop();
    
    /*In the PIC18F23K20, when uC is in I2C Master mode, the 7 lower bits of this register generate the Baud Rate*/
    //SSPADD = 19;
    /*F46K22 : Since Fclk is computed thanks to SSP1ADD, we have to set it as well. */
    SSP1ADD = 19; //SCL clock period. 
    Nop();                             
    Nop();
    Nop();
    /*F23K20 : SSPSTAT is the MSSP status register (I2C Mode)*/
    //SSPSTATbits.SMP = SET;             // disable slew rate control for standard speed mode.
    //SSPSTATbits.CKE = 0;               // disable SMBUS compliance
     
    /*F23K20 : PIR1 and PIR2 registers manage the peripheral interruptions */
    //PIR1bits.SSPIF = CLEAR;            // clear master interrupt flag, waiting for transmit/receive (SSP1IF=0, bit 3)
    //PIR2bits.BCLIF = CLEAR;            // clear collision flag
    
    /*F46K22 : SSP1STAT is one of the 2 MSSP status register. Can be used either for I2C or SPI*/
    /*Only the 1st two bytes are writable. The others are readable only.*/
    SSP1STATbits.SMP = SET; //standard speed mode
    SSP1STATbits.CKE = 0; // disable SMbus
   
    /*F46K22 : Clearing useful bits from PIR1 and PIR2 registers (MSSP1)*/
    PIR1bits.SSP1IF = CLEAR; //Master is ready to wait or transmit datas on MSSP1
    PIR2bits.BCL1IF = CLEAR; // clear collision flag of MSSP1
    
    /*F46K22 : Clearing useful bits from PIR3 (MSSP2) */
    //DO NOT DO IT !!!!! Otherwise, when waiting for the end of transmission of in LoRa function, sensor won't be detected anymore
    /*PIR3bits.BCL2IF = CLEAR; // clear collision flag of MSSP2
    PIR3bits.SSP2IF = CLEAR; // Master is ready to wait to transfer or receive datas*/
   
}

void i2c_waitForIdle() {
    /*The idea is that while the 5 lowest bits of SSPCON2 are 0, uC wait for idle*/
    /*For PIC18F23K20*/
    /*while ((SSPCON2 & 0x1F) | SSPSTATbits.RW);  // wait for idle and not writing (RCEN=0, PEN=0, RSEN=0, SEN=0, RW=0)
                                                  // all bits are exclusive*/
    
    /*FORPIC18F46K22*/
    /*At page 252 we get the answer about "Why this line of code ? " 
     "OR-ing this bit (RW from SSP1STAT register) with SEN, RSEN, PEN, RCEN or ACKEN will indicate if the MSSPx is in the idle mode" */
    /*If uC is in the loop, we know that the uC just waits to receive or transmit something*/
    while((SSP1CON2 & 0x1F) |SSP1STATbits.RW); 
}

void i2c_start(void) {
    
    /*PIC18F23K20*/
    /*PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    SSPCON2bits.SEN = ENABLE;                  // enable start condition (SEN=1, bit 0)
    while (!PIR1bits.SSPIF);                    // wait for interrupt when hardware drops SEN*/
    
    /*PIC18F46K22*/
    PIR1bits.SSP1IF = CLEAR; //We clear again this bit. It ensures the Master is ready to transmit/receive.
    SSP1CON2bits.SEN = ENABLE; //Initiate start condition on SDA1 and SCL1 pins
    while(!PIR1bits.SSP1IF); //While the transmission/reception between the master and slave is not over.
}

void i2c_repStart(void) {

    i2c_waitForIdle();                          // check for idle state
    /*PIC18F23K20*/
    /*PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    SSPCON2bits.RSEN = ENABLE;                 // enable restart condition (RSEN=1, bit 1)
    while (!PIR1bits.SSPIF);                    // wait for interrupt when hardware drops RSEN*/
    
    /*PIC18F46K22*/
    PIR1bits.SSP1IF = CLEAR; //Ensures that Master is ready to receive/transmit
    SSP1CON2bits.RSEN = ENABLE; //Initiate Repeated start condition on SCL1 and SDA1
    while(!PIR1bits.SSP1IF); //While transmission/reception between master and slave is not over
}

void i2c_stop(void) {

    i2c_waitForIdle();                          // check for idle state
    /*PIC18F23K20*/
    /*PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    SSPCON2bits.PEN = ENABLE;                  // enable stop condition (PEN = 1, bit 2)
    while (!PIR1bits.SSPIF);                    // wait for interrupt when hardware drops RSEN*/
    
    /*PIC18F46K22*/
    PIR1bits.SSP1IF = CLEAR; //As usual
    SSP1CON2bits.PEN = ENABLE; //Stop condition enabled
    while(!PIR1bits.SSP1IF); //while communicating
}

void i2c_ACK(void) {
    
    /*PIC18F23K20*/
    /*PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    SSPCON2bits.ACKDT= CLEAR;                  // clear the acknowledge data bit
    SSPCON2bits.ACKEN = ENABLE;                // enable transmission for acknowledge
    while(!PIR1bits.SSPIF);                    // wait for interrupt*/
    
    /*PIC18F46K22*/
    PIR1bits.SSP1IF = CLEAR; //As usual, makes sure Master is ready to receive/transmit
    SSP1CON2bits.ACKDT = CLEAR; //Indicate if a NAK or a ACK has to be sent
    SSP1CON2bits.ACKEN = ENABLE; //Initiate the NAK/ACK on the SDA1/SCL1 pins and transmit it.
    while(!PIR1bits.SSP1IF); //While communication between master and slave is not over
}
void i2c_NAK(void) {

    /*PIC18F23K20*/
    /*Same as above, but ACKDT to "1" will indicate that we wish to send an NAK*/
    /*PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    SSPCON2bits.ACKDT = SET;                   // set the acknowledge data bit, i.e. non-acknowledge
    SSPCON2bits.ACKEN = ENABLE;                // enable transmission for acknowledge
    while (!PIR1bits.SSPIF);                   // wait for interruption*/
    
    /*PIC18F46K22*/
    PIR1bits.SSP1IF = CLEAR; //As usual
    SSP1CON2bits.ACKDT = SET; //Here a "1" will indicate that we want to send a NAK
    SSP1CON2bits.ACKEN = ENABLE; //Allows the transmission of the NAK
    while(!PIR1bits.SSP1IF); // While communicating
}

UINT8_T i2c_read(void) {
    UINT8_T i2cReadData = 0;

    i2c_waitForIdle();                          // check for idle state
    
    /*PIC18F23K20*/
    /*PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    SSPCON2bits.RCEN = ENABLE;                 // enable receive mode (RCEN=1, bit 4)*
//    while (SSP1CON2bits.RCEN);                  // wait until hardware drops RCEN         // can be omitted
    while (!SSPSTATbits.BF);                   // wait for transmission completed (BF: Buffer Full)
    PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    i2cReadData = SSPBUF;                      // read data    (clears flag BF - Buffer Full)*/
    
    /*PIC18F46K22*/
    PIR1bits.SSP1IF = CLEAR; //As usual
    SSP1CON2bits.RCEN = ENABLE; //Master is in receive mode
    while(!SSP1STATbits.BF); //Wait for transmission completed
    PIR1bits.SSP1IF = CLEAR; //Communication over, we clear this flag.
    i2cReadData = SSP1BUF; // SSP1BUF contains the read data.
    return i2cReadData;
}

void i2c_write(UINT8_T i2cWriteData) {

    i2c_waitForIdle();                          // check for idle state
    /*F23K20*/
    /*PIR1bits.SSPIF = CLEAR;                    // clear interrupt flag
    SSPBUF = i2cWriteData;                     // set i2c buffer
    while (!PIR1bits.SSPIF);                   // wait for interrupt*/
    
    /*F46K22*/
    PIR1bits.SSP1IF = CLEAR; //As usual
    SSP1BUF = i2cWriteData; //Push the data to write in the SSP1BUF register
    while(!PIR1bits.SSP1IF); //While communication is not over
}

