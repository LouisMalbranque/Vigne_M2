#include "init_F43K22.h"


/*Set the clock used for the uC (either internal or exterior, and which frequency is used) */
void init_F46K22_CLK(void){
    
    OSCCONbits.IDLEN = 0; //uC enters sleep mode (not idle) on SLEEP instruction
    OSCCONbits.IRCF = 0b110; // Internal oscillator at 8 MHz
    OSCCONbits.OSTS = 1; //Clock defined in at the beginning of this file, in the CONFIG1H register (FOSC is set at INTI07)
    OSCCONbits.HFIOFS = 1; //HFINTOSC frequency is stable (Internal 16 MHz oscillator)
    OSCCONbits.SCS0 = 0; //SCS1 and SCS1 are the last two bits of OSCCON register.
    OSCCONbits.SCS1 = 0; // 00 tells that we have choosen the previously defined clock (FOSC in the CONFIG1H register)
}

void init_F23K20_CLK(void){
    /*F23K20 clock register*/
    OSCCON = 0b01101100;    // internal oscillator = 8 MHz (=> for UART, SPBRG1 will be set to 16 = 0x10 (thus baudrate = 117647 bps and error = +2 %)
}

/*Say which pin is either in output or input*/
void init_F46K22_IO(void){
    
     /*Set Input and Output with on the PIC18F46K22*/
     /* One input is analogic input (voltage battery)
     * 5 pins are used for serie communication (2 pins for I2C interface and 3 pins for SPI interface
     */
    //Input for battery voltage on A0
    TRISAbits.TRISA0 = 1; //battery voltage (1 is input and 0 is output)
    
    /*Input for SPI and I2C interfaces on their corresponding pins (D0,D1,D3,D4 and C3,C4 respectively)*/
    //SPI interface
    TRISDbits.TRISD0 = 1;
    TRISDbits.TRISD1 = 1;
    TRISDbits.TRISD3 = 1;
    TRISDbits.TRISD4 = 1;
    
    //I2C interface
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
}

void init_F23K20_IO(void){
    // ***** IOs configuration *****
    
    TRISA = 0b00000111;     // RA0 & RA1: analog inputs  --  RA2: digital input  --  others are outputs
    TRISB = 0x00;           // RBi: output
    // TRISB = 0b11000000;  // for debugging: PGC and PGD should be configured as inputs
    TRISC = 0b10011000;     // RC7(RX)  --  RC6(TX)  --  RC4(SDA) & RC3(SCL): must be configured as inputs
}

/*Initialize the ADC registers*/
void init_F46K22_ADC(void){
    
    /*F46K22*/
    //VREFCON0 register allows the use of a fixed interal voltage reference, here 2.048 V
    VREFCON0bits.FVREN = 1; //Allows the FVR
    VREFCON0bits.FVRS = 0b10; //Set it at 2.048V
    while(!VREFCON0bits.FVRST); //Datasheet says that the ADC internal circuit needs some stabilization after being woke up. FLag will be raised once it is done.
    
    //ADCON2 register
    ADCON2bits.ADFM = 1; //Right justified
    ADCON2bits.ACQT = 0b111; //To measure the battery voltage, we take a large acquisition time
    ADCON2bits.ADCS2 = 1; //The 3 following bits are setting the A/D clock (here Fosc/64)
    ADCON2bits.ADCS1 = 1;
    ADCON2bits.ADCS0 = 0;
    
    //ADCON0 resgister 
    ADCON0bits.ADON = 1; //Wake up the ADC
    ADCON0bits.CHS = 0b00000; //Conversion from the AN0 analog pin
    ADCON0bits.GODONE = 0;// A/D conversion not in progress
    
    /*Very useful informations at the page 293 of the datasheet*/
    //ADCON1 and related registers
    //Use of the CCP5 Special Event Trigger. Thanks to the S.E.T, an A/D conversion starts
    CCP5CONbits.CCP5M = 0b1011; //Enable the S.E.T in the CCP module
    ADCON1bits.TRIGSEL = 0; //Make the CCP starts an A/D conversion
    
    //ADC automatic flag interruption cleared
    PIR1bits.ADIF = 0;//ADC own interrupt bit in the PIR1 register must be cleared by software
    
}

void init_F23K20_ADC(void){
    /* ***** ADC configuration ***** */
    /*F23K20*/
    ADCON2 = 0b10111110;                // right justified (ADFM=1), A/D acq time = 20 TAD (ACTQ<2:0>=111), A/D clock = fOSC/64 (ADCS<2:0>=110)
    ADCON1 = 0b00000000;                // voltage ref = Vdd,Vss, 
    ADCON0 = 0b00000001;                // channel 0, stop conversion (GODONE=0), enable ADC (ADON=1)
}

/*Enable/Disable the analog input on some pins*/
void init_F46K22_AN(void){
    
    ANSELAbits.ANSA0 = 1; //Analog input on A0 (i.e digital input deleted)
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    ANSELAbits.ANSA3 = 0;
    ANSELAbits.ANSA5 = 0;
    
    //We have only digital inputs on the Di, Ci and Ei pins (SPI, I2C and SPI'S SS)
    ANSELD = 0b00000000;
    ANSELC = 0b00000000;
    ANSELE = 0b00000000;   
}

void init_F23K20_AN(void){
     /*Indicate wich pins are analog, so disable the digital input*/
    /*F23K20*/
    /*ANSELH = 0b00000000; //Manages Bx pins 
    ANSEL = 0b00000011;    //Manages Ei and Ai pins .  Analog input on RA0 and RA1 (disable digital input buffer)*/
}


void init_F46K22_TMR(void){
       
    /* ***** Timer0 configuration ***** */
    //Same for F23K20 and F46K22
    T0CON = 0b10000111;                 // Timer on, 16-bit, internal clock, prescaler = 256
    /* End of Timer0 configuration */
    
    /* ***** Interrupts configuration ***** */ 
    //Same for F23K20 and F46K22
    INTCON = 0b00100000;                // enable TMR0 overflow interrupt
}

void init_F23K20_TMR(void){
       
    /* ***** Timer0 configuration ***** */
    //Same for F23K20 and F46K22
    T0CON = 0b10000111;                 // Timer on, 16-bit, internal clock, prescaler = 256
    /* End of Timer0 configuration */
    
    /* ***** Interrupts configuration ***** */ 
    //Same for F23K20 and F46K22
    INTCON = 0b00100000;                // enable TMR0 overflow interrupt
}

/*void __interrupt () TMR0Interrupt() {

  if (INTCONbits.TMR0IF == 1)     // check if interrupt is triggered by TMR0
  {
      ++timer_tick;
      // jumper removed => RA2 = 0 => long period between 2 transmissions
      // jumper inserted => RA2 = 1 => short period between 2 transmissions
      //if (((PORTAbits.RA2 == 0)&&(timer_tick >= long_period)) || ((PORTAbits.RA2 == 1)&&(timer_tick >= short_period)))
      if(timer_tick >= our_period)
      {
        send_data = 1;
        timer_tick = 0;
      } 

      INTCONbits.TMR0IF = 0;     // clear interrupt flag
  }
}*/