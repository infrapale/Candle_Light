/* 
 * File:   main.c
 * Author: tom_h
 *
 * Created on October 24, 2020, 10:24 AM
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = EXTRC_CLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define bitset(var, bitno) ((var) |= 1UL << (bitno))
#define bitclr(var, bitno) ((var) &= ~(1UL << (bitno)))

#define _XTAL_FREQ 8000000 //Specify the XTAL crystal FREQ
#define DUTY_LEN   8
const uint16_t duty_16[16] = {
    //1111110000000000
    //5432109876543210    
    0b0000000000000000,
    0b0000000000000001,
    0b0000000100000001,
    0b0010000100001000,
    0b0100100010001000,
    0b0100010001000100,
    0b0100101010100100,
    0b0101010010101010,
    0b0101010101010101,
    0b1101101101101100,
    0b1101101101101101,
    0b1101111101101101,
    0b1110111101110110,
    0b1111011110111110,
    0b1101111110111111,
    0b1111111111111111,
};
const uint8_t led_set_cntr[8] = {8,14,25,30,41,47,55,61};
uint8_t led_cntr[8];
uint8_t duty_indx[8];
uint8_t duty_delta[8];
/*
 * 
 */

void main() //The main function
{ 
    int8_t i=0;
    int8_t i_d=1;
    uint8_t j=0;
    uint8_t led_indx;
    uint16_t n=0;
    uint16_t mask = 0;
    uint8_t  tick_cntr = 0;
    uint8_t  idx;
    
    TRISD=0X00; //Instruct the MCU that the PORTB pins are used as Output.
    PORTD=0X00; //Make all output of RB3 LOW
    for (i=0;i<8;i++){
        led_cntr[i] = led_set_cntr[i];
        duty_indx[i] = 0;
        duty_delta[i] = 1;        
    }
    while(1)    //Get into the Infinite While loop
    {
        for (tick_cntr=0;tick_cntr < 16;tick_cntr++){
            for (led_indx = 0; led_indx<8; led_indx++){
                if (--led_cntr[led_indx] == 0){
                    led_cntr[led_indx] = led_set_cntr[led_indx];
                    if (duty_delta[led_indx] == 1) {
                        if (duty_indx[led_indx] < 15){
                            duty_indx[led_indx]++;
                        } else {
                            duty_delta[led_indx] = 2;
                        }   
                    } else {
                        if (duty_indx[led_indx] > 1){
                            duty_indx[led_indx]--;
                        } else {
                            duty_delta[led_indx] = 1;
                        }   
                    }
                }    
                mask = 1 << tick_cntr;  //
                idx = duty_indx[led_indx];
                if((duty_16[idx] & mask) != 0){
                    //PORTD |= (1<<led_indx);  
                    bitset(PORTD,led_indx);
                }
                else {
                    //PORTD &= ~(1<<led_indx);
                    bitclr(PORTD,led_indx);
                }
                    //__delay_ms(1);
            }
        }
    }
}

