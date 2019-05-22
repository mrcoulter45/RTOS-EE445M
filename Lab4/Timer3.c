// Timer3.c
// Runs on LM4F120/TM4C123
// Use Timer3 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// March 20, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
  Program 7.5, example 7.6

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
#include <stdint.h>
#include "OS.h"
#include "UART.h"
#include "../inc/tm4c123gh6pm.h"

void (*PeriodicTask3)(void);   // user function

unsigned long PeriodicThread2Start[SAMPLES];
unsigned long PeriodicThread2Finish[SAMPLES];
unsigned long Periodic2StartPlace = 0;
unsigned long Periodic2FinishPlace = 0;

void dumpPeriodic2(void)
{
	for(int i = 0; i < SAMPLES; i+=1)
	{
		UART_OutChar('['); UART_OutUDec(PeriodicThread2Start[i]); UART_OutString(" - "); UART_OutUDec(PeriodicThread2Finish[i]); UART_OutChar(']');
		UART_OutString("\n\r");
	}
}

void resetPeriodic2(void)
{
	Periodic2StartPlace = 0;
	Periodic2FinishPlace = 0;
}

unsigned long filterWorkT3 = 0;
#define PERIODT3 TIMER3_TAILR_R
unsigned long lastTimeT3;
unsigned long maxJitterT3;
#define JITTERSIZE 64
unsigned long const jitterSizeT3 = JITTERSIZE;
unsigned long jitterHistogramT3[JITTERSIZE]={0,};
extern unsigned long MaxJitter;
void JitterT3(void)
{
	unsigned long input;  
  unsigned long jitter = 0;                    // time between measured and expected, in us
	input = 1;
	unsigned long thisTime = OS_Time();
  filterWorkT3++;        // calculation finished
  if(filterWorkT3>1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(lastTimeT3,thisTime);
		// unsigned long diff = thisTime - lastTimeT3;

    if(diff>PERIODT3){
      jitter = (diff-PERIODT3+4)/8;  // in 0.1 usec
    }else{
      jitter = (PERIODT3-diff+4)/8;  // in 0.1 usec
    }
    if(jitter > maxJitterT3){
      maxJitterT3 = jitter; // in usec
    }       // jitter should be 0
    if(jitter >= jitterSizeT3){
      jitter = JITTERSIZE-1;
    }
		if(jitter > MaxJitter)
		{
			MaxJitter = jitter;
		}
    jitterHistogramT3[jitter]++; 
  }
	
	lastTimeT3 = thisTime;
}

unsigned long Timer3Jitter()
{
	return maxJitterT3;
}

// ***************** Timer3_Init ****************
// Activate Timer3 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer3_Init(void(*task)(void), unsigned long period, unsigned long priority){
  SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
  PeriodicTask3 = task;          // user function
  TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAILR_R = period-1;    // 4) reload value
  TIMER3_TAPR_R = 0;            // 5) bus clock resolution
  TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
  TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  //  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x20000000; // 8) priority 1
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|((priority&0x07)<<29); // 8) set priority
  // interrupts enabled in the main program after all devices initialized
  // vector number 51, interrupt number 35
  NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
  TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
}

 void Timer3A_Handler(void){
   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
   TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
	 JitterT3();
	 if(Periodic2StartPlace < SAMPLES)
	 {
		 PeriodicThread2Start[Periodic2StartPlace++] = OS_Time();
	 }
   (*PeriodicTask3)();                // execute user task
	 if(Periodic2FinishPlace < SAMPLES)
	 {
		 PeriodicThread2Finish[Periodic2FinishPlace++] = OS_Time();
	 }
   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
 }
