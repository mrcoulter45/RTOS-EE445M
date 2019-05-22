// Michael Coulter mrc3463
// Jake Klovenski jdk2595
// Lab 1
// main.c
// Runs on LM4F120/TM4C123
// Provide a function that initializes Timer0A to trigger ADC
// SS3 conversions and request an interrupt when the conversion
// is complete.
// Daniel Valvano
// May 3, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// center of X-ohm potentiometer connected to PE3/AIN0
// bottom of X-ohm potentiometer connected to ground
// top of X-ohm potentiometer connected to +3.3V through X/10-ohm ohm resistor
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "ADC.h"
#include "PLL.h"
#include "UART.h"
#include "OS.h"
#include "ST7735.h"
#include "Timer3.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void blink(void) 
{
    GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
}

void dummy(void){

}

void heartbeatInit(void) 
{
    SYSCTL_RCGCGPIO_R |= 0x00000020;         // activate port F
    GPIO_PORTF_DIR_R |= 0x04;                // make PF2 out (built-in LED)
    GPIO_PORTF_AFSEL_R &= ~0x04;             // disable alt funct on PF2
    GPIO_PORTF_DEN_R |= 0x04;                // enable digital I/O on PF2
                         // configure PF2 as GPIO
    GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
    GPIO_PORTF_AMSEL_R = 0;                  // disable analog functionality on PF
}

uint16_t strcmp(char* str1, char* str2)
{
    uint32_t i = 0;
    while(str1[i]!=0 && str2[i]!=0)
    {
        if(str1[i] != str2[i])
        {
            return -1;
        }
        i++;
    }
    if(str1[i] == str2[i])
    {
        return 0;
    }
    return 1;

}


char* reverse(char* str, int length) 
{
    char* one = str;
    char* two = str + length - 1;
    while(one < two)
    {
        char hold = *one;
        *one = *two;
        *two = hold;
        one++;
        two--;
    }
    return str;
}
char* itoa(uint32_t num, char* str, int base) 
{
    int i = 0;
    if(num == 0)
    {
        str[i] = '0';
        str[i + 1] = 0;
        return str;
    }
    
    while(num != 0)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }
    str[i] = 0;
    reverse(str, i);
    return str;
}

//debug code
//
// This program periodically samples ADC0 channel 0 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.


int main(void){
    PLL_Init(Bus50MHz);                      // 80 MHz system clock
    SYSCTL_RCGCGPIO_R |= 0x00000020;         // activate port F
    //ADC_Open(0);
    //uint16_t dataBuffer[64];
    //ADC_Collect(0, 8000000, dataBuffer, 64);
    GPIO_PORTF_DIR_R |= 0x04;                // make PF2 out (built-in LED)
    GPIO_PORTF_AFSEL_R &= ~0x04;             // disable alt funct on PF2
    GPIO_PORTF_DEN_R |= 0x04;                // enable digital I/O on PF2
                                             // configure PF2 as GPIO
    GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
    GPIO_PORTF_AMSEL_R = 0;                  // disable analog functionality on PF
    GPIO_PORTF_DATA_R &= ~0x04;              // turn off LED
    EnableInterrupts();
    UART_Init();              // initialize UART
    heartbeatInit();        //initialize heartbeat
    
    //LCD init
    ST7735_InitR(INITR_REDTAB);
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_DivideScreen();
    OS_AddPeriodicThread(*blink, 500, 1);//passed function will run on the Timer 3 interrupt
    int adc_val;
    char userInput[20];
    while(1){
        UART_OutString("--> ");
        UART_InString(userInput,20);
        UART_OutString("\n\r");
        if(strcmp(userInput,"ADC_Read") == 0)
        {
            UART_OutString("ADC Value:");
            char val[10];
            itoa(ADC_In(), val, 10);
            UART_OutString(val);
        }
        else if(strcmp(userInput,"ADC_Collect") == 0){
        	UART_OutString("Enter Period: ");
        	uint32_t user_period = UART_InUDec();
            UART_OutString("\r\n");
            UART_OutString("Number of Samples: ");
            uint32_t user_num_samples = UART_InUDec();
            uint16_t userDataBuffer[user_num_samples];
            ADC_Collect(0, user_period, userDataBuffer, user_num_samples);
        }
        else if(strcmp(userInput,"Hello") == 0)
        {
            UART_OutString("Hi!");
        }
        else if(strcmp(userInput, "Time") == 0)
        {
            uint32_t time = OS_ReadPeriodicTime();
            char time_str[10];
            itoa(time,time_str,10);
            UART_OutString("Time = ");
            UART_OutString(time_str);
        }
        else if((strcmp(userInput, "Screen") == 0) || (strcmp(userInput, "screen") == 0) || (strcmp(userInput, "screen ") == 0))
        {
            UART_OutString("Which one? 1 or 2 ");
            uint32_t screen = UART_InUDec();
            UART_OutString("\r\n");
            if(screen > 2)
            {
                UART_OutString("That was not an option.\r\n");
                continue;
            }
            UART_OutString("What line? 0-3 ");
            uint32_t line = UART_InUDec();
            UART_OutString("\r\n");
            if(line > 3)
            {
                UART_OutString("That is not a valid option.\r\n");
                continue;
            }
            UART_OutString("What would you like to say?\r\n");
            UART_InString(userInput,20);
            ST7735_Message(screen,line,userInput,0,ST7735_WHITE);
            
        }
        else if(strcmp(userInput, "Clear Time") == 0)
        {
            OS_ClearPeriodicTime();
            UART_OutString("Time reset to 0."); 
        }
        else if(strcmp(userInput, "Clear Screen") == 0)
        {
            ST7735_FillScreen(ST7735_BLACK);
            ST7735_DivideScreen();
            UART_OutString("Screen cleared");
        }
        else 
        {
            UART_OutString("That is not a recognized command.");
        }
        UART_OutString("\n\r");
    }
}

