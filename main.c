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
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.80.0
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

#include "mcc_generated_files/mcc.h"
//#define FCY 16000000UL    //16MHz
#define FCY 31000UL     //31KHz

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
    //RA4 = 0;        // no use
    while (1)
    {
        // Add your application code
        NOP();
        SLEEP();
        /*
         * eliminate chattering & Press the power button for 3 seconds to turn on.
         */ 
        for(i=0;i<12;i++){  //wait for 3 seconds maximum, blink LED
            RA1 = 0;
            __delay_ms(150);
            RA1 = 1;        // LED ON
            __delay_ms(100);
        }        
  
        if (RA5 & (!RC3)){      // if SW On and RPi=OFF
            RC2 = 0;        // Power On
            for(i=0;i<120;i++){  //wait for 120 seconds maximum, blink LED
                RA1 = 0;
                __delay_ms(700);
                RA1 = 1;        // LED ON
                __delay_ms(300);
                if (RC3 == 1) break;
            }
        } else if (RC5 & RC3) {  // if SW On and RPi=ON
            for(i=0;i<240;i++){  //wait for 120 seconds maximum, blink LED
                RA1 = 0;
                __delay_ms(300);
                RA1 = 1;        // LED ON
                __delay_ms(200);
                if (RC3 == 0) break;
            }
        }

        if (RC3 == 0){
            __delay_ms(1000);            
            RA1 = 0;        // LED OFF
            __delay_ms(1000);
            RC2 = 1;        // Power Off
        }
    }
}
/**
 End of File
*/