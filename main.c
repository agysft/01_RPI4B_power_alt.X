/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.0
        Device            :  PIC16F1503
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include <xc.h>
#include "mcc_generated_files/mcc.h"

#define FCY 16000000UL    //16MHz
//#define FCY 31000UL     //31KHz
#include <stdio.h>
#include <string.h>

/******** I2C ********/
void I2C_init(){
    SSP1CON1 = 0b00101000;  // Enables the serial port and configures the SDAx and SCLx pins as the source of the serial port pins, I2C Master mode
    ANSELC = 0;             // 0:Digital
    TRISCbits.TRISC0 = 1;   // 1:input ; When enabled, the SDAx and SCLx pins must be configured as inputs.
    TRISCbits.TRISC1 = 1;
    SSP1CON3 =  0b00000000;
    SSP1ADD =   9;          // 16MHz/(400KHz x4) -1 = 16000000/(400000 x4)-1 = 9
}

void i2c_START(){
    PIR1bits.SSP1IF = 0;
    SSP1CON2bits.SEN = 1;
    while (PIR1bits.SSP1IF == 0) {}
    PIR1bits.SSP1IF = 0;
}

void i2c_TXDAT(char data){
    PIR1bits.SSP1IF = 0;
    SSP1BUF = data;
    while (PIR1bits.SSP1IF == 0) {}
    PIR1bits.SSP1IF = 0;
}

void i2c_STOP(){
    SSP1CON2bits.PEN = 1;
    while (SSP1CON2bits.PEN == 1) {}
    PIR1bits.SSP1IF = 0;
}

/******** I2C LCD ********/
#define LCD_ADD 0x3E
bool LCD = false;

int Is_LCD_Connected(){
    int exist_the_lcd = 1;  // 0: exist the LCD, 1: No LCD
    
    if ( (RC0 | RC1) != 0){ // if no pull-up
        i2c_START();
        i2c_TXDAT(LCD_ADD<<1);
        exist_the_lcd = SSP1CON2bits.ACKSTAT;   // 1: No responded (= no ACK)
        if ( exist_the_lcd == 0){
            i2c_TXDAT(0x00);
            i2c_TXDAT(0x00);    // dummy
            i2c_STOP();
        }
    }
    return exist_the_lcd;
}

void writeLCDCommand(char t_command){
    i2c_START();
    i2c_TXDAT(LCD_ADD<<1);
    i2c_TXDAT(0x00);
    i2c_TXDAT(t_command);
    i2c_STOP();
    __delay_us(30);     //Instruction Execution Time 14.3-26.3us
}

void LCD_Init(){
    __delay_ms(400);
    writeLCDCommand(0x38);
    writeLCDCommand(0x39);
    writeLCDCommand(0x14);
    writeLCDCommand(0x75);// contast LSB setting ; 0b0111 xxxx
    writeLCDCommand(0x50);// 5V=0b0101 00xx, 3V=0b0101 01xx,  xx=contrast MSB
    writeLCDCommand(0x6C);
    __delay_ms(250);
    writeLCDCommand(0x38);
    writeLCDCommand(0x0C);
    writeLCDCommand(0x01);
    __delay_us(1100);        //Instruction Execution Time 0.59-1.08ms (550:NG, 600:GOOD)
}

void LCD_xy(uint8_t x, uint8_t y){
    writeLCDCommand(0x80 + 0x40 * y + x);
}

void LCD_str2(char *c) {
    unsigned char wk;
    i2c_START();
    i2c_TXDAT(LCD_ADD<<1);
    for (int i=0;i<8;i++){
        wk = c[i];
        if  (wk == 0x00) {break;}
        i2c_TXDAT(0xc0);
        i2c_TXDAT(wk);
    }
    i2c_STOP();
    __delay_us(30);
}

void LCD_clear(){
    writeLCDCommand(0x01);
    __delay_us(1100);        //Instruction Execution Time 0.59-1.08ms (550:NG, 600:GOOD)
}

bool POffFlug = false;
int IntSource = 0;
/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    int i;
    RC2 = 1;        // Power Off
    RA1 = 0;        // LED OFF
    
    I2C_init();
    if ( Is_LCD_Connected() == 0 ) {
        LCD = true; 
    } else {
        LCD = false;
    }
    
    if (LCD) LCD_Init();
    if (LCD) { LCD_xy(0,0); LCD_str2("HELLO"); }
    if (LCD) { LCD_xy(0,1); LCD_str2("PI-SW"); }
    
    while (1)
    {
        NOP();
        SLEEP();
        if (LCD) LCD_clear();

        if ( IntSource == 2 ){
            if (LCD) { LCD_xy(0,1); LCD_str2("INT RPi"); }
            for(i=0;i<6;i++){  //wait for 1.5 seconds, blink LED
                RA1 = ~RA1; __delay_ms(100);
                RA1 = ~RA1; __delay_ms(150);
            }
            if (POffFlug){
                for(i=0;i<12;i++){  //wait for 3 seconds, blink LED
                    RA1 = ~RA1; __delay_ms(100);
                    RA1 = ~RA1; __delay_ms(150);
                }
                if (LCD) { LCD_xy(0,0); LCD_str2("Pow OFF1"); }
                RA1 = 0;        // LED OFF
                RC2 = 1;        // Power Off
                POffFlug = false;
            } else {
                if (RC3){
                    POffFlug = true;
                    RA1 = 0;        // LED OFF
                    if (LCD) { LCD_xy(0,1); LCD_str2("POffFlug"); }
                } else {
                    for(i=0;i<120;i++){ //wait for 120 seconds maximum, blink LED
                        RA1 = 0; __delay_ms(700);
                        RA1 = 1; __delay_ms(300);
                        if (RC3) {
                            if (LCD) { LCD_xy(0,1); LCD_str2("RPi RBT"); }
                            break; // RC3==1 when reboot
                        }
                    }
                    if (!RC3){
                        if (LCD) { LCD_xy(0,0); LCD_str2("Pow OFF2"); }
                        RA1 = 0;        // LED OFF
                        RC2 = 1;        // Power Off
                        POffFlug = false;
                    }
                }
            }
        }
        if ( (IntSource == 1) && (!POffFlug) ){
            if (LCD) { LCD_xy(0,1); LCD_str2("INT SW"); }
            for(i=0;i<4;i++){  //wait for 1 seconds, blink LED
                RA1 = ~RA1; __delay_ms(100);
                RA1 = ~RA1; __delay_ms(150);
            }
            if ( RA5 & (!RC3) ){  // if SW==On and RPi==OFF
                if (LCD) { LCD_xy(0,0); LCD_str2("Pow ON"); }
                RC2 = 0;        // Power On
                for(i=0;i<120;i++){ //wait for 120 seconds maximum, blink LED
                    RA1 = 0; __delay_ms(700);
                    RA1 = 1; __delay_ms(300);
                    if (RC3 == 1) {
                        if (LCD) { LCD_xy(0,1); LCD_str2("RPi ON"); }
                        break;
                    }
                }
            }
        }
        IntSource = 0;
    }
}

void CLC1_ISR(void)
{
    // Clear the CLC interrupt flag
    PIR3bits.CLC1IF = 0;
    IntSource = 1;
}

void CLC2_ISR(void)
{
    // Clear the CLC interrupt flag
    PIR3bits.CLC2IF = 0;
    IntSource = 2;
}

/**
 End of File
*/