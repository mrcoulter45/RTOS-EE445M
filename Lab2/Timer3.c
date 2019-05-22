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
#include "../inc/tm4c123gh6pm.h"

void (*PeriodicTask)(void);   // user function

// ***************** Timer3_Init ****************
// Activate Timer3 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
// void Timer3_Init(void(*task)(void), unsigned long period){
//   SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
//   PeriodicTask = task;          // user function
//   TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
//   TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
//   TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
//   TIMER3_TAILR_R = 40000-1;    // 4) reload value
//   // TIMER3_TAILR_R = period-1;    // 4) reload value
//   TIMER3_TAPR_R = 0;            // 5) bus clock resolution
//   TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
//   TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
//   // NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x80000000; // 8) priority 4
//   NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x20000000; // 8) priority 1
// // interrupts enabled in the main program after all devices initialized
// // vector number 51, interrupt number 35
//   NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
//   TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
// }

// void Timer3A_Handler(void){
//   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
//   TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
//   (*PeriodicTask)();                // execute user task
//   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
// }

// void Timer2_Init(void(*task)(void), unsigned long period){
//   SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
//   PeriodicTask = task;          // user function
//   TIMER2_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
//   TIMER2_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
//   TIMER2_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
//   TIMER2_TAILR_R = 40000-1;    // 4) reload value
//   // TIMER3_TAILR_R = period-1;    // 4) reload value
//   TIMER2_TAPR_R = 0;            // 5) bus clock resolution
//   TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
//   TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
//   // NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x80000000; // 8) priority 4
//   NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x20000000; // 8) priority 1
// // interrupts enabled in the main program after all devices initialized
// // vector number 51, interrupt number 35
//   NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
//   TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
// }

unsigned long TimerCount;
void Timer2_Init(void(*task)(void), unsigned long period){ 
  unsigned long volatile delay;
  SYSCTL_RCGCTIMER_R |= 0x04;   // 0) activate timer2
  PeriodicTask = task;
  delay = SYSCTL_RCGCTIMER_R;
  TimerCount = 0;
  TIMER2_CTL_R = 0x00000000;   // 1) disable timer2A
  TIMER2_CFG_R = 0x00000000;   // 2) 32-bit mode
  TIMER2_TAMR_R = 0x00000002;  // 3) periodic mode
  TIMER2_TAILR_R = 0x000fffff-1;   // 4) reload value
  TIMER2_TAPR_R = 0;           // 5) clock resolution
  TIMER2_ICR_R = 0x00000001;   // 6) clear timeout flag
  TIMER2_IMR_R = 0x00000001;   // 7) arm timeout
  NVIC_PRI5_R = (NVIC_PRI5_R&0x0FFFFFFF)|0x20000000; 
// 8) priority 1
  NVIC_EN0_R = 1<<23;          // 9) enable IRQ 23 in
  TIMER2_CTL_R = 0x00000001;   // 10) enable timer2A
}


// void Timer3A_Handler(void){
//   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
//   TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
//   (*PeriodicTask)();                // execute user task
//   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
// }

// void Timer3A_Handler(void){
//   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
//   TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
//   (*PeriodicTask)();                // execute user task
//   //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
// }

void Timer2A_Handler(void){
  TIMER2_ICR_R = 0x00000001;  // acknowledge
	uint32_t current = TIMER2_TAR_R;
  TimerCount++;
  (*PeriodicTask)();
  /*TIMER2_CTL_R = 0x00000000;   // 1) disable timer2A
  TIMER2_CFG_R = 0x00000000;   // 2) 32-bit mode
  TIMER2_TAMR_R = 0x00000002;  // 3) periodic mode
  TIMER2_TAILR_R = 40000-1;   // 4) reload value
  TIMER2_TAPR_R = 0;           // 5) clock resolution
  TIMER2_ICR_R = 0x00000001;   // 6) clear timeout flag
  TIMER2_IMR_R = 0x00000001;   // 7) arm timeout
  NVIC_PRI5_R = (NVIC_PRI5_R&0x0FFFFFFF)|0x20000000; 
// 8) priority 1
  NVIC_EN0_R = 1<<23;          // 9) enable IRQ 23 in
  TIMER2_CTL_R = 0x00000001;   // 10) enable timer2A*/
// run some background stuff here
}
