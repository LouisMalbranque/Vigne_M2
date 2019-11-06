#include "general.h"
#include "LoRa_com.h"


//uint8_t txBuffer[256];

void init_LORA_communication(){
    
    
    InitRFLoRaPins();           // configure pins for RF Solutions LoRa module   
    SPIInit();                  // init SPI   
    ResetRFModule();            // reset the RF Solutions LoRa module (should be optional since Power On Reset is implemented)
    
    AntennaTX();                // connect antenna to module output
    
    // put module in LoRa mode (see SX1272 datasheet page 107)
    WriteSXRegister(REG_OP_MODE, FSK_SLEEP_MODE);       // SLEEP mode required first to change bit n°7
    WriteSXRegister(REG_OP_MODE, LORA_SLEEP_MODE);      // switch from FSK mode to LoRa mode
    WriteSXRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // STANDBY mode required fot FIFO loading
    __delay_ms(100);
    GetMode();
    
    // initialize the module
    InitModule();
}

void load_FIFO_with_temp_humidity_voltage(uint8_t id_trame, uint8_t id_reseau, uint8_t id_node, double battery_voltage, double temperature, double humidity){
        
        uint8_t b_temperature = (uint8_t)(((temperature + 30) * 255) / 70);
        uint8_t b_humidity = (uint8_t)((humidity * 255) / 100);
        uint8_t b_battery_voltage = (uint8_t)((battery_voltage * 255) / 3);
        uint8_t txMsg[PAYLOAD_LENGTH_1];
        
        WriteSXRegister(REG_FIFO_ADDR_PTR, ReadSXRegister(REG_FIFO_TX_BASE_ADDR));      // FifiAddrPtr takes value of FifoTxBaseAddr
        WriteSXRegister(REG_PAYLOAD_LENGTH_LORA, PAYLOAD_LENGTH_1);                       // set the number of bytes to transmit (PAYLOAD_LENGTH is defined in RF_LoRa868_SO.h)

        txMsg[0] = 1;
        txMsg[1] = id_node;
        txMsg[2] = id_reseau;
        txMsg[3] = b_battery_voltage;
        txMsg[4] = b_temperature;
        txMsg[5] = b_humidity;
        for (int i = 0; i < PAYLOAD_LENGTH_1; i++) {
            WriteSXRegister(REG_FIFO, txMsg[i]);         // load FIFO with data to transmit  
        }
}

void set_TX_and_transmit(void){
     // set mode to LoRa TX
        WriteSXRegister(REG_OP_MODE, LORA_TX_MODE);
        __delay_ms(100);                                    // delay required to start oscillator and PLL

}

void wait_for_transmission(){
   // wait end of transmission
    uint8_t reg_val = ReadSXRegister(REG_IRQ_FLAGS);
    while (reg_val & 0x08 == 0x00) {                    // wait for end of transmission (wait until TxDone is set)
        reg_val = ReadSXRegister(REG_IRQ_FLAGS);
    }
}

void reset_IRQs(){
    // reset flags register
    WriteSXRegister(REG_IRQ_FLAGS, 0xFF);           // clear flags: writing 1 clears flag
}

