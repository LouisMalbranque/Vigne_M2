/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:  LoRa_com.h
 * Author: LG/LM/LB/JMC/BRS
 * Comments: From TX_LORA.c
 * Revision history: 31/10/19
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef _LORA_COM_H
#define	_LORA_COM_H

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "spi.h"
#include "uart.h"
#include "SX1272.h"
#include "RF_LoRa_868_SO.h"

void init_LORA_communication();
void load_FIFO_with_temp_humidity_voltage(uint8_t id_trame, uint8_t id_reseau, uint8_t id_node, double battery_voltage, double temperature, double humidity);

void set_TX_and_transmit(void);
void wait_for_transmission(void);
void reset_IRQs(void);

#endif	/* _LORA_COM_H */

