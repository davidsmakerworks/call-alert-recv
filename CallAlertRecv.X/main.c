/*
 * Remote Call-Holding Alert Receiver and Display Module
 * Copyright (c) 2019 David Rice
 * 
 * Processor: PIC16F18325
 * 
 * The following macros must be defined on the XC8 command line or in project properties:
 * _XTAL_FREQ - CPU speed in Hz (Fosc) - must be 32000000 for this driver
 * RF_CHANNEL - the channel to use for the RF module (example: 0x10U)
 * RF_RX_ADDR - the name of the variable containing the receive address for the RF module (example: address1)
 * MAX_LED_INDEX - the maximum index of the connected to the driver (i.e., number of LEDs - 1)
 * 
 * Additionally, exactly one of the following must be defined (without a value) to specify 
 * the type of LEDs connected to the driver:
 * WS2811
 * WS2812B
 * 
 * Drivers used:
 * NRF24L01P
 * 
 * Peripheral usage:
 * MSSP1 - Drives WS281x LED strip (SPI master mode)
 * MSSP2 - Communication with RF module (SPI master mode)
 * Timer1 - Generates timestamp for automatic shut-off
 * Timer2 - PWM clock source for WS281x protocol
 * PWM5 - Generates PWM signal for WS281x protocol
 * CLC1 - Combines MSSP1 and PWM5 signals to produce WS2812x protocol
 * 
 * Pin assignments:
 * RA4 - WS2812x data output from CLC
 * RC0 - RF module CE
 * RC1 - RF module CSN
 * RC2 - RF module SCK
 * RC3 - RF module MOSI
 * RC4 - RF module MISO
 * RC5 - RF module IRQ
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18325 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "nRF24L01P.h"
#include "nRF24L01P-cfg.h"

#ifndef RF_CHANNEL
#error RF_CHANNEL must be defined
#endif

#ifndef RF_RX_ADDR
#error RF_RX_ADDR must be defined
#endif

#ifndef MAX_LED_INDEX
#error MAX_LED_INDEX must be defined
#endif

#if _XTAL_FREQ != 32000000
#error _XTAL_FREQ must be defined as 32000000 (this driver requires Fosc = 32 MHz)
#endif

/* 
 * Macros for putting color values at the appropriate index
 * Note that the WS2812B uses GRB format instead of RGB
 */
#ifdef WS2811
#define RED(x)   ((x) * 3)
#define GREEN(x) ((x) * 3) + 1
#define BLUE(x)  ((x) * 3) + 2
#else
#ifdef WS2812B
#define RED(x)   ((x) * 3) + 1
#define GREEN(x) ((x) * 3)
#define BLUE(x)  ((x) * 3) + 2
#else
#error LED type must be defined
#endif
#endif

#define ADDR_LEN        5
#define PAYLOAD_WIDTH   1

#define BUTTON_A_PAYLOAD    0xA6
#define BUTTON_B_PAYLOAD    0xB5
#define BUTTON_C_PAYLOAD    0xC0
#define BUTTON_D_PAYLOAD    0xD2

const uint8_t strip1_addr[ADDR_LEN] = { 'T', 'N', 'E', 'T', 0xAA };
const uint8_t strip2_addr[ADDR_LEN] = { 'T', 'N', 'E', 'T', 0xBB };

const uint8_t cool_blue_values[15] = { 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF, 0x7F, 0x3F, 0x1F, 0x0F, 0x00, 0x00, 0x00 };

typedef enum {
    OFF,
    COOL_BLUE,
    PULSE_RED,
    FLASH_RED,
    RED_YELLOW
} STATE;

uint8_t color_data[(MAX_LED_INDEX + 1) * 3];

/* 
 * Standard port initialization
 * Later functions will change some of these settings 
 */
void init_ports(void) {
    /* Disable all analog features */
    ANSELA = 0x00;
    ANSELC = 0x00;
    
    /* Pull all outputs low except RC1 (RF_CSN) */
    LATA = 0x00;
    LATC = _LATC_LATC1_MASK;
    
    /* Set all ports to output except RC4 (SDI2) and RC5 (RF_IRQ) */
    TRISA = 0x00;
    TRISC = _TRISC_TRISC4_MASK |
            _TRISC_TRISC5_MASK;
    
    /* Set TTL on RC4 (SDI2) and RC5 (RF_IRQ) due to 3.3V output from RF module */
    INLVLCbits.INLVLC4 = 0;
    INLVLCbits.INLVLC5 = 0;
}

/* Initialize SPI modules that will drive the RF module and CLC */
void init_spi(void) {
    /* SPI2 used to drive RF module */
    
    /* Set MSSP2 to SPI Master mode using baud rate generator */
    SSP2CON1bits.SSPM = 0b1010;
    
    /* 1 MHz at Fosc = 32 MHz */
    SSP2ADD = 7;
    
    /* Transmit data on active-to-idle transition */
    SSP2STATbits.CKE = 1;
    
    /* Enable MSSP2 */
    SSP2CON1bits.SSPEN = 1;
    
    /* SPI1 used only to drive CLC */
    SSP1CON1bits.SSPM = 0b0011; /* Set SPI mode with CLK = T2_match/2 */
    SSP1CON1bits.SSPEN = 1; /* Enable MSSP */
}

/* Initialize Timer1 to drive auto shut-off and Timer2 to be PWM clock source */
void init_timers(void) {
    T1CONbits.TMR1CS = 0b11; /* LFINTOSC */
    T1CONbits.T1CKPS = 0b00; /* 1:1 */
    
    PR2 = 4; /* 0.625 uSec at Fosc = 32 MHz */
    T2CONbits.TMR2ON = 1; /* Enable Timer2 */
}

/* Initialize PWM generator for WS281x zero-bit pulses */
void init_pwm(void) {
    PWM5DCH = 1;
    PWM5DCL = 0;
    
    PWM5CONbits.PWM5EN = 1; /* Enable PWM generator */
}

/* Set up CLC to output (SCK && SDO) || (nSDO && SCK && PWM) */
void init_clc(void) {
    CLC1SEL0bits.LC1D1S = 0b10011; /* CLC1 input 1 is SDO1 */
    CLC1SEL1bits.LC1D2S = 0b10010; /* CLC1 input 2 is SCK1 */
    CLC1SEL2bits.LC1D3S = 0b10000; /* CLC1 input 3 is PWM5OUT */
    
    CLC1GLS0 = 0x00;
    CLC1GLS1 = 0x00;
    CLC1GLS2 = 0x00;
    CLC1GLS3 = 0x00; /* Gate behavior is undefined at power-on so must be set to zero */
    
    CLC1GLS0bits.LC1G1D1T = 1; /* SDO input to AND gate 1 */
    
    CLC1GLS1bits.LC1G2D2T = 1; /* SCK input to AND gate 1 */
    
    /* nSDO && SCK = n(SDO || nSCK) */
    CLC1GLS2bits.LC1G3D1T = 1;
    CLC1GLS2bits.LC1G3D2N = 1; /* SDO || nSCK input to AND gate 2 */
    
    CLC1GLS3bits.LC1G4D3T = 1; /* PWM5OUT input to AND gate 2 */
    
    CLC1POL = 0x00; /* Clear all inversion bits */
    CLC1POLbits.LC1G3POL = 1; /* Gate 3 n(SDO || nSCK) is inverted to obtain (nSDO && SDK) */
    
    CLC1CONbits.LC1EN = 1; /* Enable CLC1 */
}

void init_pps(void) {
    bool state;
    
    /* Preserve global interrupt state and disable interrupts */
    state = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    
    /* Unlock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    /* SCK2 on RC2 */
    RC2PPS = 0b11010;
    SSP2CLKPPS = 0b10010;
    
    /* SDI2 on RC4 */
    SSP2DATPPS = 0b10100;
    
    /* SDO2 on RC3 */
    RC3PPS = 0b11011;
    
    /* CLC1OUT on RA4 */
    RA4PPS = 0b00100;
    
    /* Lock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    /* Restore global interrupt state */
    INTCONbits.GIE = state;
}

void init_rf(void) {
    /* Allow for maximum possible RF module startup time */
    __delay_ms(100);
    
    /* Set 1-byte payload width */
    nrf24_write_register(NRF24_RX_PW_P0, PAYLOAD_WIDTH);
    
    /* Set RF power to 0 dBm and data rate to 1 Mbit/Sec */
    nrf24_write_register(NRF24_RF_SETUP, NRF24_RF_PWR_0DBM);
    
    /* Set 5-byte address width */
    nrf24_write_register(NRF24_SETUP_AW, NRF24_AW_5);
    
    /* Set initial RF channel */
    nrf24_write_register(NRF24_RF_CH, RF_CHANNEL);
    
    /* Enable receive on pipe 0 only */
    nrf24_write_register(NRF24_EN_RXADDR, NRF24_ERX_P0);
    
    /* Set receive address  */
    nrf24_write_register_multi(NRF24_RX_ADDR_P0, RF_RX_ADDR, ADDR_LEN);
    
    /* Mask RX_DR interrupt on RF module, enable CRC, power up RF module in transmit-standby mode */
    nrf24_write_register(NRF24_CONFIG, NRF24_MASK_TX_DS | NRF24_MASK_MAX_RT | NRF24_EN_CRC | NRF24_PWR_UP | NRF24_PRIM_RX);
    
    /* Clear any pending RF module interrupts */
    nrf24_write_register(NRF24_STATUS, NRF24_TX_DS | NRF24_MAX_RT | NRF24_RX_DR);
}

uint8_t transfer_spi(uint8_t data) {
    SSP2BUF = data;
    
    while (!SSP2STATbits.BF);
    
    data = SSP2BUF;
    
    return data;
}

void send_led_data(void) {
    uint16_t current_index;
    
    for (current_index = 0; current_index < (MAX_LED_INDEX + 1) * 3; current_index++) {
        SSP1BUF = color_data[current_index];
        
        while (!SSP1STATbits.BF);
    }
}

void all_red(uint8_t val) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led++) {
        color_data[RED(current_led)] = val;
        color_data[GREEN(current_led)] = 0x00;
        color_data[BLUE(current_led)] = 0x00;
    }
}

void all_green(void) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led++) {
        color_data[RED(current_led)] = 0x00;
        color_data[GREEN(current_led)] = 0x7F;
        color_data[BLUE(current_led)] = 0x00;
    }
}

void all_blue(void) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led++) {
        color_data[RED(current_led)] = 0x00;
        color_data[GREEN(current_led)] = 0x00;
        color_data[BLUE(current_led)] = 0x7F;
    }
}

void all_off(void) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led++) {
        color_data[RED(current_led)] = 0x00;
        color_data[GREEN(current_led)] = 0x00;
        color_data[BLUE(current_led)] = 0x00;
    }
}

void all_magenta(void) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led++) {
        color_data[RED(current_led)] = 0x7F;
        color_data[GREEN(current_led)] = 0x00;
        color_data[BLUE(current_led)] = 0x7F;
    }
}

void red_yellow(bool red_first) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led += 2) {
        if (red_first) {
            color_data[RED(current_led)] = 0xFF;
            color_data[GREEN(current_led)] = 0x00;
            color_data[BLUE(current_led)] = 0x00;
            
            color_data[RED(current_led + 1)] = 0xFF;
            color_data[GREEN(current_led + 1)] = 0x7F;
            color_data[BLUE(current_led + 1)] = 0x00;
        } else { 
            color_data[RED(current_led)] = 0xFF;
            color_data[GREEN(current_led)] = 0x7F;
            color_data[BLUE(current_led)] = 0x00;
            
            color_data[RED(current_led + 1)] = 0xFF;
            color_data[GREEN(current_led + 1)] = 0x00;
            color_data[BLUE(current_led + 1)] = 0x00;
        }     
    }
}

void cool_blue(uint8_t frame) {
    uint16_t current_led;
    
    for (current_led = 0; current_led < MAX_LED_INDEX + 1; current_led++) {
        color_data[RED(current_led)] = 0x00;
        color_data[GREEN(current_led)] = 0x00;
        color_data[BLUE(current_led)] = cool_blue_values[frame];
        
        frame++;
        
        if (frame > 14) {
            frame = 0;
        }
    }
}

void main(void) {
    uint8_t buf;
    uint8_t red_val = 0x0F;
    uint8_t rollovers;
    uint8_t cool_blue_frame = 0;
    bool red_increasing = true;
    
    bool red_first = true;
    
    STATE state;
    
    bool update_needed = false;
    
    /* Initialize peripherals */
    init_ports();
    init_pps();
    init_timers();
    init_pwm();
    init_clc();
    init_spi();
    init_rf();
    
    NRF24_CE_ACTIVE();
    
    all_off();
    update_needed = true;
    
    state = OFF;
    
    while (1) {
        if (update_needed) {
            send_led_data();
            update_needed = false;
        }
        
        if (!NRF24_IRQ) {
            nrf24_read_payload(&buf, PAYLOAD_WIDTH);
            
            switch(buf) {
                case BUTTON_A_PAYLOAD:
                    state = OFF;
                    T1CONbits.TMR1ON = 0;
                    PIR1bits.TMR1IF = 0;
                    break;
                    
                case BUTTON_B_PAYLOAD:
                    state = COOL_BLUE;
                    rollovers = 0;
                    TMR1 = 0;
                    T1CONbits.TMR1ON = 1;
                    break;
                    
                case BUTTON_C_PAYLOAD:
                    state = PULSE_RED;
                    red_val = 0xFF;
                    rollovers = 0;
                    TMR1 = 0;
                    T1CONbits.TMR1ON = 1;
                    break;
                    
                case BUTTON_D_PAYLOAD:
                    state = RED_YELLOW;
                    rollovers = 0;
                    TMR1 = 0;
                    T1CONbits.TMR1ON = 1;
                    break;
                    
                default:
                    all_off();
                    update_needed = true;
            }
            
            nrf24_write_register(NRF24_STATUS, NRF24_RX_DR);            
        }
        
        if (state != OFF && PIR1bits.TMR1IF) {
            rollovers++;
            
            PIR1bits.TMR1IF = 0;
            
            if (rollovers > 27) {
                T1CONbits.TMR1ON = 0;
                state = OFF;
            } 
        }
        
        switch (state) {
            case OFF:
                all_off();
                update_needed = true;
                break;
            
            case COOL_BLUE:
                cool_blue(cool_blue_frame);
                
                cool_blue_frame++;
                
                if (cool_blue_frame > 14) {
                    cool_blue_frame = 0;
                }
                
                update_needed = true;
                
                __delay_ms(20);
                break;
                
            case PULSE_RED:
                all_red(red_val);
                
                if (red_increasing) {
                    red_val = red_val + 5;
                    
                    if (red_val > 250) {
                        red_increasing = false;
                    }
                } else {
                    red_val = red_val - 5;
                    
                    if (red_val < 0x0F) {
                        red_increasing = true;
                    }
                }
                
                update_needed = true;
                
                __delay_ms(20);
                break;
                
            case FLASH_RED:
                all_red(red_val);
                
                if (red_val == 0xFF) {
                    red_val = 0x00;
                } else {
                    red_val = 0xFF;
                }
                
                update_needed = true;
                __delay_ms(500);
                break;
               
            case RED_YELLOW:
                red_yellow(red_first);
                
                red_first = !red_first;
                
                update_needed = true;
                
                __delay_ms(500);
                break;
                
            default:
                all_green();
                update_needed = true;
                
        }
    }
}
