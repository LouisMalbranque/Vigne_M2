#include "EEPROM.h"


void writeEEPROM(UINT8_T address, UINT8_T data){
        // 0x22 -> adresse 0x01
    EEADR = address;               // set address
    EEADRH = 0x00;
    EEDATA = data;              // set data
    EECON1bits.EEPGD = 0;       // access EEPROM memory (not Flash memory)
    EECON1bits.CFGS = 0;        // access Flash or EEPROM memory (not configuration registers)
    EECON1bits.WREN = 1;        // allows write to Flash or EEPROM memory
    INTCONbits.GIE = 0;         // inhibits interrupts (recommended during Flash or EEPROM write cycles)
    //EECON2 = 0x55;
    //EECON2 = 0xAA;
    //EECON1bits.WR = 1;

    asm("MOVLW 0x55");
    asm("MOVWF EECON2");                // write 0x55 to EECON2
    asm("MOVLW 0xAA");
    asm("MOVWF EECON2");                // write 0xAA to EECON2
    asm("BSF EECON1, 1, 0");            // set WR bit

    while(EECON1bits.WR){};     // wait until WR is cleared (done by hardware at the completion of the write cycle)
    //while(!PIR2bits.EEIF);
    PIR2bits.EEIF = 0;          // EEIF is set by hardware at completion of write cycle, so clear it (even if not used here))
    asm("NOP");
    INTCONbits.GIE = 1;         // re-activate interrupts (if necessary)
}


UINT8_T readEEPROM(UINT8_T address){
    EEADR = address;
    EEADRH = 0x00;
    EECON1bits.EEPGD = 0;           // EEPGD = 0 (access data EEPROM memory, not Flash program memory)
    EECON1bits.CFGS = 0;            // CFGS = 0 (access data EEPROM memory, not configuration registers)
    EECON1bits.RD = 1;              // initiate EEPROM read
    return EEDATA;
}