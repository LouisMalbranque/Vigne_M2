/* 
 * Author: jmc
 *
 * Created on 12 july 2016
 */

// ****************************************
// ****  MEASURED CIRCUIT CONSUMPTION: ****
// ****  7 mA under 3.3 V              ****
// ****************************************

#define USE_AND_MASKS

// PIC18F23K20 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*****/
// CONFIG1H
#pragma config FOSC = INTIO7    // Oscillator Selection bits (Internal oscillator block, CLKOUT function on OSC2)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (Power up timer enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
//#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
//#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
//#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
//#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
//#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
//#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)



// define time between two transmissions on SIGFOX network
// the timer cycle is 65536 * 256 * (4 / 8MHz) = 8.388608 seconds
// (16-bit, prescaler = 256, fosc = 8 MHz)
// long period (jumper removed) => 1 hour between 2 transmissions = 429 cycles
// short period (jumper inserted) => 15 mn between 2 transmissions = 107 cycles
#define long_period    429
#define short_period   107
#define our_period 73
#define our_period2 4
//#define long_period    4    // for debugging
//#define short_period   1    // for debugging

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "general.h"
#include "LoRa_com.h"
#include "i2c.h"
#include "EEPROM.h"

#include "init_F43K22.h"


    UINT8_T header1;    // header = header1 header0 = frame number
    UINT8_T header0;    // from 0x0000 to 0x9999  (BCD coded))
    
    UINT8_T bat_voltage1;   // battery voltage = bat_voltage1 bat_voltage0 (BCD coded)
    UINT8_T bat_voltage0;   // from 0x0000 to 0x9999  (BCD coded))
                            // bat_voltage1 holds volt units, bat_voltage0 holds tenths and hundreds
                            // i.e. 0x06 0x51 for 6.51 V
    
    UINT8_T bat_voltage_unit; 
    UINT8_T bat_voltage_decim_part;
    UINT16_T accu_v;
    UINT16_T accu_v2;
    UINT8_T reg_val;

    UINT8_T panel_voltage1;     // solar panel voltage = panel_voltage1 panel_voltage0 (BCD coded)
    UINT8_T panel_voltage0;     // from 0x0000 to 0x9999  (BCD coded))
                                // panel_voltage1 holds  volt units, voltage 0 holds tenths and hundreds
                                // i.e. 0x17 0x60 for 17.60 V
    
    uint8_t txBuffer[256]; //Loaded with content of txMsg. txBuffer is the array sent by the LoRa
    uint8_t reg_val;
    //uint8_t txMsg[]=""; //Message which is wanted to be sent
    uint8_t id_node = 1; //Sensor ID
    uint8_t id_trame = 1; // Trame ID
    uint8_t id_reseau = 231;//Network ID in which the sensor belongs to
    

    double volts=0.0;                     // voltage in Volts
    //double battery_voltage=0.0;             //Taking into account the resistive divider
    double humidity = 0.0;

    UINT16_T volts_cV;               // voltage in centiVolts
    UINT16_T ADC_result; 
    UINT16_T temp_AT = 0;
    uint8_t battery_voltage_int=3;
    int16_t i;
    BOOL send_data;
    UINT16_T timer_tick;
    

    
    UINT16_T temperatureHL=0;
    UINT16_T humidityHL=0;
    double temperature = 0.0;
    double volts_divider = 0.0;
    
    uint8_t txMsg[6];
    uint8_t idT=0;
    uint8_t idR=0xFF;
    uint8_t idN=0xFF;
    uint8_t var1=0xFF;
    uint8_t var2=0xFF;
    double battery_voltage = 3.0;
    
    void measure_battery(void);
    void measure_humidity_temp_HIH6021(void);




int main(int argc, char** argv) {
   
    //1st step : Set the clock (OSCCON register)
    //2nd step : Set the I/O thanks to TRISx register
    //3rd step : Set the ADC registers (ADCON0, ADCON1 and ADCON2)
    //4th step : Disable the digital input with ANSELA,ANSELC,ANSELD and ANSELE
    //5th step : Set the timer and interruption registers
    header1 = 0;
    header0 = 0;
    bat_voltage1 = 0;
    bat_voltage0 = 0;
    //uint8_t txMsg[6];
    
    UARTInit(19200);            // init UART @ 19200 bps
    
    init_F46K22();
    
    // ***** LoRa & I2C configuration *****
    init_LORA_communication();
    i2c_init();
    ei();                         // enable interrupt
    
    measure_battery();
    
    // get data from eeprom
    id_node = readEEPROM(0x00);
    id_reseau = readEEPROM(0x01);
    
    // for fun, to test EEPROM
    writeEEPROM(0x00, id_node + 1);
    writeEEPROM(0x01, id_reseau + 2);
    
    
    // load fifo
    load_FIFO_with_init_values(0, id_reseau, id_node, battery_voltage);
    
    // set Lora mode in TX mode to send data
    set_TX_and_transmit();
        
    // wait for the end of transmission
    wait_for_transmission();
        
    // reset interrupts 
    reset_IRQs();
    
    while(1){
        measure_humidity_temp_HIH6021();        
        measure_battery();
        
        // load fifo with data
        load_FIFO_with_temp_humidity_voltage(1, id_reseau, id_node, battery_voltage, temperature, humidity);
        
        // set Lora mode in TX mode to send data
        set_TX_and_transmit();
        
        // wait for the end of transmission
        wait_for_transmission();
        
        // reset interrupts 
        reset_IRQs();
      
        __delay_ms(200);        // delay is required before checking mode: it takes some time to go from TX mode to STDBY mode

        // sleep for some time
        __delay_ms(10000);
        __delay_ms(10000);
        __delay_ms(10000);
        __delay_ms(10000);
        __delay_ms(10000);
        __delay_ms(10000);
        //Trame 1 : id_trame, id_reseau , id_noeud (0 pour trame requete), v_bat
        //Trame 2 : id_trame, id_reseau, id_noeud (1 pour celle ci), voltage, temp,humidité
    }
    return (EXIT_SUCCESS);
}

void measure_battery(void)
{
//    float volts;                     // voltage in Volts
//    UINT16_T volts_cV;                  // voltage in cVolts
//    unsigned int ADC_result;         // 16-bit 2's complement - to store the result of ADC conversion
    
    //Channel has already been choosen
    //Conversion is started by the Special Event Trigger, or manually by us ? 
    ADCON0bits.GO = 1; // Set manually
    //while(!ADCON0bits.GO) //Waits for the Special Event Trigger to set the GO bit
    while (ADCON0bits.GO);//Polling on the GO bit, not the best solution, as it blocks the uC.
    PIR1bits.ADIF = 0; //ADC own interrupt bit in the PIR1 register must be cleared by software
  
    
    //Result is stored in a 16 bits long variable. ADRESH and ADRESL receive the result from the ADC
    //ADRESH and ADRESL are too low, for 3V, we get 0x02 and 0xC6. We should have 0x03 and 0xFF !  (i.e 1023 for 3V) => Elementary, my dear Watson !
    ADC_result = (ADRESH << 8 )|ADRESL;
    // 583 --> 6V
    
    
    /* ***********IF YOU NEED A RESISTIVE DIVIDER TO LOWER THE VOLTAGE FROM THE BATTERY TO THE uC, READ THIS !!!!!************     */
    
    // convert ADC value into volts
    // 1 LSB is (3.3 / 1023) volt
    // take into account input resistive divider: 330k and 220k
    // (so 7V battery voltage is translated to 2.8V)
    // => voltage resolution is (3.3 / 1023) * (550 / 220) = 8 mV
    // => max voltage on input = 3.3 * (550 / 220) = 8.25 V
//    volts = (3.3 / 1023) * (550 / 220) * ADC_result;    // example: volts = 6.51
    // remarque: la ligne ci-dessus donne un résultat erroné (testé avec le debugger))
    // => il faut la simplifier par la ligne ci-dessous
    //volts = (8.75 / 1023) * ADC_result;
    
    
    /* **************Just multiply the ADC_result by the 1 LSB value (the one from the F.V.R *********** */
    volts = (2.048/1023)*ADC_result; 
    battery_voltage = volts *(2.115); //Real measured voltage, roughly approximated
    /*volts = (8.8 / 1023) * ADC_result;
    volts = (3.3/1023);
    volts = volts * (560/220);
    volts = volts * ADC_result;*/
    //volts = (8.32 / 1023) * ADC_result;   */      // adjusted relation based on practical test
                                                // (takes into account components deviation)
    /*volts_cV = (UINT16_T)(volts * 100);                         // example: volts_cV = 651
    bat_voltage1 = volts_cV / 100;                                  // example: bat_voltage1 = 0x06
    bat_voltage0 = ((volts_cV / 10) % 10 << 4) | (volts_cV % 10);   // example: bat_voltage0 = 0x51*/
    
    //The goal now is to separate the battery voltage in 2 separated frames : One stands for the units , and one will hold .x and the last .0x
    
    accu_v = (UINT16_T)(battery_voltage * 100);
    accu_v2 = (UINT16_T)(battery_voltage*100);
    bat_voltage_unit = (accu_v/100); // Unit number of battery voltage
    //Careful, if you divide directly on accu_v, you'll get only some 0, as it has been casted in UINT8_T. New variable is needed.
    bat_voltage_decim_part = ((accu_v2/10)%10 << 4) | (accu_v2 % 10);
    
     /*An A/D conversion takes around 20 uS to perform (page 452 of the datasheet) , so at least 25 to 30 uS (for safety)
     are requiered between 2 conversions */
    __delay_ms(0.03); 
}


/*uC starts communication with HIH. It sends 4 bytes. 2 stands for temperature and 2 for the humidity.
 * A computing is then used on these 4 bytes in order to have the matching temp and humidity
 */
void measure_humidity_temp_HIH6021(void){
    UINT8_T humidity_HIH6021_H = 0;
    UINT8_T humidity_HIH6021_L = 0;
    UINT8_T temperature_HIH6021_H = 0;
    UINT8_T temperature_HIH6021_L = 0;
    
    
    i2c_start();//I2C bus initialization
    /*Wake up the sensor*/
    i2c_write((0x27) << 1 | WRITE );//Reaching the slave's address, and put a WRITE
    i2c_stop();
    
    __delay_ms(40); // mandatory delay to let the sensor mesure (36.65 ms according to datasheet)
    
    /*Once started, we retrieve the datas measured*/
    i2c_start();
    i2c_write((0x27) << 1 | READ);//First byte to send, which is the I2C address and a "1" bit at last (which match a READ state)
    //__delay_ms(38);//Delay in order to let the sensor write its data into the register
    Nop();
    humidity_HIH6021_H = i2c_read(); //uC reads the first byte written by sensor
    i2c_ACK();//Needed before reading next byte
    humidity_HIH6021_L = i2c_read();//uC reads the second byte written by sensor
    i2c_ACK();//Needed before reading next byte
    temperature_HIH6021_H = i2c_read();//uC reads the first byte written by sensor
    i2c_ACK();//Needed before reading next byte
    temperature_HIH6021_L = i2c_read();//uC reads the second byte written by sensor
    i2c_NAK();//NAK is written by master after last byte is received
    i2c_stop();//stop the communication between uC and sensor
   // __delay_ms(50);
    
    /*The code aftwards will be all the treatement about the received datas*/
    humidityHL = ((humidity_HIH6021_H & 0b00111111) << 8) | humidity_HIH6021_L;//We delete the 2 bits status, then shift the 6 remaining bits to the left and concatenate the 6 L bits 
    temperatureHL = (temperature_HIH6021_H << 6 | temperature_HIH6021_L);//1st byte shifted by 8 bits on the left, and datasheet says do "not care" about the last 2 bytes.
    temperatureHL >> 2;
                                                                         //so we do not care
    
    
    // convert 14bit value into double value 
    humidity = ( ( (double)humidityHL ) / 16382 )*100; //Use the humidity conversion formula provided by the datasheet
    temperature = ( ( ( (double)temperatureHL ) /16382 )*165 ) - 40;

}