/* 
 * File:   EEPROM.h
 * Author: louis
 *
 * Created on 6 novembre 2019, 15:12
 */

#ifndef EEPROM_H
#define	EEPROM_H

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "general.h"

#define ID_NODE_ADDRESS 0x00;
#define ID_RESEAU_ADDRESS 0x01;


#ifdef	__cplusplus
extern "C" {
#endif

    UINT8_T readEEPROM(UINT8_T address);                // returns content of EEPROM at address
    void writeEEPROM(UINT8_T address, UINT8_T data);    // sets data into EEPROM at address


#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

