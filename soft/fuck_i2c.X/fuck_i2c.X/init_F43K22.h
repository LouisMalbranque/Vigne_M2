/* 
 * File:   init_F43K22.h
 * Author: louis
 *
 * Created on 6 novembre 2019, 11:51
 */

#ifndef INIT_F43K22_H
#define	INIT_F43K22_H

#include <pic18f46k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

    void init_F46K22_CLK(void);
    void init_F23K20_CLK(void);
    void init_F46K22_IO(void);
    void init_F23K20_IO(void);
    void init_F46K22_ADC(void);
    void init_F23K20_ADC(void);
    void init_F46K22_AN(void);
    void init_F23K20_AN(void);
    void init_F46K22_TMR(void);
    void init_F23K20_TMR(void);
    void __interrupt () TMR0Interrupt();


#ifdef	__cplusplus
}
#endif

#endif	/* INIT_F43K22_H */

