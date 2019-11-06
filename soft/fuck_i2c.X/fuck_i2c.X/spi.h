/*
 * File:   spi.h
 * Authors: LAL & JMC
 *
 * Created (LAL) on 9 avril 2015
 * Modified (JMC) on 18 mai 2017
 */

#ifndef _SPI_H
#define	_SPI_H

#include <xc.h>
#include "general.h"
#include <stdint.h>             // with this inclusion, the XC compiler will recognize standard types such as uint8_t or int16_t 
                                // (so, their definition in "general.h" is useless)

// PIC18F46K22 SPI master mode
// for MSSP n°2:    SCK is D0
//                  MISO is D1
//                  MOSI is D4
//



/*SPI pin properties for PIC18F46K22*/
#define SPI_SCK_DIR             TRISDbits.TRISD0
#define SPI_MISO_DIR            TRISDbits.TRISD1
#define SPI_MOSI_DIR            TRISDbits.TRISD4

/*SPI pin properties for PIC18F23K20*/
// Slave Select is wired on E0 (PIC18F46K22)

#define SPI_SS_DIR              TRISEbits.TRISE0
#define SPI_SS_LAT              LATEbits.LATE0

#define SPI_SS_DISABLE          OUTP_HIGH
#define SPI_SS_ENABLE           OUTP_LOW


void SPIInit(void);                                                         // init SPI in master mode
void SPITransfer (UINT8_T data_out);                                        // send a byte
UINT8_T SPIReceive (UINT8_T data_out);                                      // receive a byte and send another byte

#endif	/* _SPI_H */

