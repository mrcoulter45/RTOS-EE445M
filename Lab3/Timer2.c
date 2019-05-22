// Timer2.c
// Runs on LM4F120/TM4C123
// Use TIMER2 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// May 5, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
  Program 7.5, example 7.6

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
#include <stdint.h>
#include "OS.h"
#include "UART.h"
#include "../inc/tm4c123gh6pm.h"

unsigned long PeriodicThread1Start[SAMPLES];
unsigned long PeriodicThread1Finish[SAMPLES];
unsigned long Periodic1StartPlace = 0;
unsigned long Periodic1FinishPlace = 0;

void dumpPeriodic1(void)
{
	for(int i = 0; i < SAMPLES; i+=1)
	{
		UART_OutChar('['); UART_OutUDec(PeriodicThread1Start[i]); UART_OutString(" - "); UART_OutUDec(PeriodicThread1Finish[i]); UART_OutChar(']');
		UART_OutString("\n\r");
	}
}

void resetPeriodic1(void)
{
	Periodic1StartPlace = 0;
	Periodic1FinishPlace = 0;
}

void (*PeriodicTask2)(void);   // user function

unsigned long filterWork = 0;
#define PERIODT2 TIMER2_TAILR_R
unsigned long lastTime;
unsigned long maxJitter;
#define JITTERSIZE 64
unsigned long const jitterSize = JITTERSIZE;
unsigned long jitterHistogram[JITTERSIZE]={0,};
extern unsigned long MaxJitter;

void JitterT2(void)
{
	unsigned long input;  
  unsigned long jitter = 0;                    // time between measured and expected, in us
	input = 1;
	unsigned long thisTime = OS_Time();
  filterWork++;        // calculation finished
  if(filterWork>1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(lastTime,thisTime);
		// unsigned long diff = thisTime - lastTime;

    if(diff>PERIODT2){
      jitter = (diff-PERIODT2+4)/8;  // in 0.1 usec
    }else{
      jitter = (PERIODT2-diff+4)/8;  // in 0.1 usec
    }
    if(jitter > maxJitter){
      maxJitter = jitter; // in usec
    }       // jitter should be 0
    if(jitter >= jitterSize){
      jitter = JITTERSIZE-1;
    }
		if(jitter > MaxJitter)
		{
			MaxJitter = jitter;
		}
    jitterHistogram[jitter]++; 
  }
	
	lastTime = thisTime;
}

unsigned long Timer2Jitter()
{
	return maxJitter;
}
unsigned long TimerCount;
void Timer2_Init(void(*task)(void), unsigned long period, unsigned long priority){ 
  unsigned long volatile delay;
  SYSCTL_RCGCTIMER_R |= 0x04;   // 0) activate timer2
  PeriodicTask2 = task;
  delay = SYSCTL_RCGCTIMER_R;
  TimerCount = 0;
  TIMER2_CTL_R = 0x00000000;   // 1) disable timer2A
  TIMER2_CFG_R = 0x00000000;   // 2) 32-bit mode
  TIMER2_TAMR_R = 0x00000002;  // 3) periodic mode
  TIMER2_TAILR_R = period-1;   // 4) reload value
  TIMER2_TAPR_R = 0;           // 5) clock resolution
  TIMER2_ICR_R = 0x00000001;   // 6) clear timeout flag
  TIMER2_IMR_R = 0x00000001;   // 7) arm timeout
  //NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x20000000; 
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|((priority&0x07)<<29); 
// 8) priority 4
  NVIC_EN0_R = 1<<23;          // 9) enable IRQ 23 in
  TIMER2_CTL_R = 0x00000001;   // 10) enable timer2A
}

void Timer2A_Handler(void){
  TIMER2_ICR_R = 0x00000001;  // acknowledge
	JitterT2();
	if(Periodic1StartPlace < SAMPLES)
	 {
		 PeriodicThread1Start[Periodic1StartPlace++] = OS_Time();
	 }
   (*PeriodicTask2)();                // execute user task
	 if(Periodic1FinishPlace < SAMPLES)
	 {
		 PeriodicThread1Finish[Periodic1FinishPlace++] = OS_Time();
	 }
// run some background stuff here
}

//pg 725 "register map" explains what all the timer registers do
void WTIMER0_INIT(void){
  SYSCTL_RCGCWTIMER_R |= 0x01; //activates Wide Timer 0
	int delay = SYSCTL_RCGCWTIMER_R;
	delay = SYSCTL_RCGCWTIMER_R;
	delay = SYSCTL_RCGCWTIMER_R;
  WTIMER0_CTL_R = 0;       // 1) disable WTIMER0 during setup
  WTIMER0_CFG_R = 0;       //For a 32/64-bit wide timer, this value selects the 64-bit timer configuration. (pg 728)
  //WTIMER0_SYNC_R = 0x3000; //
  WTIMER0_TAMR_R = 0x12;   //bit 4 - timer counts up. bits 1:0 = 0x02 - periodic timer mode
  //TAILR and TBILR concatenated make up the value in which this timer will reach a timeout event... about 7,312 years
  WTIMER0_TAILR_R = 0xffffffff;  
  WTIMER0_TBILR_R = 0xffffffff;
  WTIMER0_TAPR_R = 0;      // 5) bus clock resolution
  WTIMER0_TBPR_R = 0;      // 5) bus clock resolution
  WTIMER0_ICR_R = 1;       // 6) clear WTIMER0A timeout flag
  WTIMER0_IMR_R = 0;       //disable interrupts
  WTIMER0_CTL_R |= 0x001;  // bit 0 - enable Timer A. bit 8 - enable timer B.
}
