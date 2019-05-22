#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "Timer3.h"

#define RELOAD_50MHz 50000

long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void (*OSTask)(void);   // user function
uint32_t function_period;
uint32_t time = 0;
uint32_t function_priority;

void OS_Handler(void){//runs every time Timer3 interrupts
  time++;
  if(time % function_period == 0)
  {
	  (*OSTask)();
  }
}

int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority)	
{
  long sr = StartCritical(); 
  OSTask = task;          // user function
  function_priority = priority;
  function_period = period;
  Timer3_Init(*OS_Handler, RELOAD_50MHz);
  EndCritical(sr);
  return 0;
}

void OS_ClearPeriodicTime(void)
{
	time = 0;
}

uint32_t OS_ReadPeriodicTime(void)
{
	return time;
}




