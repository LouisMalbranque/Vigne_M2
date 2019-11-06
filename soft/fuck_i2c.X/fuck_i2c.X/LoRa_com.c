#include "general.h"
#include "LoRa_com.h"


//uint8_t txBuffer[256];

void init_LORA_communication(UINT16_T baud_rate){
    
    UARTInit(baud_rate);            // init UART @ 19200 bps
    InitRFLoRaPins();           // configure pins for RF Solutions LoRa module   
    SPIInit();                  // init SPI   
    ResetRFModule();            // reset the RF Solutions LoRa module (should be optional since Power On Reset is implemented)
    
    AntennaTX();                // connect antenna to module output
    
    // put module in LoRa mode (see SX1272 datasheet page 107)
    UARTWriteStrLn("set mode to LoRa standby");

    WriteSXRegister(REG_OP_MODE, FSK_SLEEP_MODE);       // SLEEP mode required first to change bit n°7
    WriteSXRegister(REG_OP_MODE, LORA_SLEEP_MODE);      // switch from FSK mode to LoRa mode
    WriteSXRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // STANDBY mode required fot FIFO loading
    __delay_ms(100);
    GetMode();
    
    // initialize the module
    UARTWriteStrLn("initialize module ");
    InitModule();
    
    // for debugging purpose only: check configuration registers content
    CheckConfiguration();
}

void load_FIFO_with_temp_humidity_voltage(double temperature,double humidity, double battery_voltage,uint8_t *txBuffer,uint8_t id_node, uint8_t id_reseau,uint8_t id_trame){
    
    int i=0;
    uint8_t b_temperature = (uint8_t)(((temperature + 30) * 255) / 70);
    uint8_t b_humidity = (uint8_t)((humidity * 255) / 100);
    uint8_t b_battery_voltage = (uint8_t)((battery_voltage * 255) / 3);
    uint8_t txMsg[6];
    txMsg[0] = id_trame;
    txMsg[1] = id_node;
    txMsg[2] = id_reseau;
    txMsg[3] = b_battery_voltage;
    txMsg[4] = b_temperature;
    txMsg[5] = b_humidity;
        
    // load FIFO with data to transmit
    strcpy(( char* )txBuffer, ( char* )txMsg);          // load txBuffer with content of txMsg
                                                        // txMsg is a table of constant values, so it is stored in Flash Memory
                                                        // txBuffer is a table of variables, so it is stored in RAM
        
    UARTWriteStrLn(" ");
    UARTWriteStrLn("step 1: load FIFO");
    WriteSXRegister(REG_FIFO_ADDR_PTR, ReadSXRegister(REG_FIFO_TX_BASE_ADDR));      // FifiAddrPtr takes value of FifoTxBaseAddr
    WriteSXRegister(REG_PAYLOAD_LENGTH_LORA, PAYLOAD_LENGTH);                       // set the number of bytes to transmit (PAYLOAD_LENGTH is defined in RF_LoRa868_SO.h)

    for (i = 0; i < PAYLOAD_LENGTH; i++) {
        WriteSXRegister(REG_FIFO, txBuffer[i]);         // load FIFO with data to transmit  
    }
}

void set_TX_and_transmit(void){
     // set mode to LoRa TX
        UARTWriteStrLn(" ");
        UARTWriteStrLn("step 2: set mode to LoRa TX");
        WriteSXRegister(REG_OP_MODE, LORA_TX_MODE);
        __delay_ms(100);                                    // delay required to start oscillator and PLL
        GetMode();
}

void has_transmitted(uint8_t reg_val){
   // wait end of transmission
    reg_val = ReadSXRegister(REG_IRQ_FLAGS);
    while (reg_val & 0x08 == 0x00) {                    // wait for end of transmission (wait until TxDone is set)
        reg_val = ReadSXRegister(REG_IRQ_FLAGS);
    }
    UARTWriteStrLn(" ");
    UARTWriteStrLn("step 3: TxDone flag set");
        
    __delay_ms(200);        // delay is required before checking mode: it takes some time to go from TX mode to STDBY mode
    GetMode();              // check that mode is back to STDBY 
}

void reset_IRQs(uint8_t reg_val){
    int i=0;
    // reset all IRQs
    UARTWriteStrLn(" ");
    UARTWriteStrLn("step 4: clear flags");
    reg_val = ReadSXRegister(REG_IRQ_FLAGS);
    UARTWriteStr("before clear: REG_IRQ_FLAGS = 0x");
    UARTWriteByteHex(reg_val);
        
    WriteSXRegister(REG_IRQ_FLAGS, 0xFF);           // clear flags: writing 1 clears flag

    // check that flags are actually cleared (useless if not debugging)
    reg_val = ReadSXRegister(REG_IRQ_FLAGS);
    UARTWriteStr("after clear: REG_IRQ_FLAGS = 0x");
    UARTWriteByteHex(reg_val);
        
    // wait before next transmission
    for (i = 0; i < 4; i++) {
         __delay_ms(500);  
    }
}

