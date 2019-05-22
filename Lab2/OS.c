#include "OS.h"
#include "PLL.h"
#include "Timer3.h"
#include "ST7735.h"
#include "UART.h"
#include "../inc/tm4c123gh6pm.h"

// Assembly functions
void OS_DisableInterrupts(void);   
void OS_EnableInterrupts(void);
//void Systick_HandlerPt2(void);
void SysTick_Handler(void);
void StartOS(void);
TCB *RunPt;
TCB *NextRunPt;

unsigned long periodicThreadPeriod;
void (*periodicTask)(void);

unsigned long time = 0;
#define NUMTHREADS 8
#define STACKSIZE 100

long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value

TCB tcbs[NUMTHREADS];
int32_t Stacks[NUMTHREADS][STACKSIZE];//array of stack pointers
int first_thread = 1;
int thread_index = 0;
int32_t thread_IDs[NUMTHREADS];
int thread_IDs_index = 0; //points to next unused thread ID
int num_threads_active = 0;

#define FIFOSIZE   32         // size of the FIFOs (must be power of 2)
#define FIFOSUCCESS 1         // return value on success
#define FIFOFAIL    0         // return value on failure

unsigned long OS_Fifo[FIFOSIZE];
int fifo_head;
int fifo_tail;
Sema4Type fifo_sema,fifo_get_sema;

//pg 177
//sets up initial stack
void SetInitialStack(int i){
	tcbs[i].sp = &Stacks[i][STACKSIZE-16]; //thread stack pointer
	Stacks[i][STACKSIZE-1]  = 0x01000000;  //thumb bit
	//Stacks[i][STACKSIZE-2] contains the PC
	Stacks[i][STACKSIZE-3]  = 0x14141414;  //R14
	Stacks[i][STACKSIZE-4]  = 0x12121212;  //R12
	Stacks[i][STACKSIZE-5]  = 0x03030303;  //R3
	Stacks[i][STACKSIZE-6]  = 0x02020202;  //R2
	Stacks[i][STACKSIZE-7]  = 0x01010101;  //R1
	Stacks[i][STACKSIZE-8]  = 0x00000000;  //R0
	Stacks[i][STACKSIZE-9]  = 0x11111111;  //R11
	Stacks[i][STACKSIZE-10] = 0x10101010;  //R10
	Stacks[i][STACKSIZE-11] = 0x09090909;  //R9
	Stacks[i][STACKSIZE-12] = 0x08080808;  //R8
	Stacks[i][STACKSIZE-13] = 0x07070707;  //R7
	Stacks[i][STACKSIZE-14] = 0x06060606;  //R6
	Stacks[i][STACKSIZE-15] = 0x05050505;  //R5
	Stacks[i][STACKSIZE-16] = 0x04040404;  //R4
}

int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority){
    //sets pointer to next and previous threads for each initialized thread
    //sets sp and PC to thread task in respective TCB object
    //set initial thread to run
    //protect against critical sections
    int32_t status;
    status = StartCritical();
    if(first_thread == 1){
    	for(int i = 0; i < NUMTHREADS; i += 1){
    		thread_IDs[i] = i;
		}
    	tcbs[thread_IDs[thread_IDs_index]].next_TCB = &tcbs[thread_IDs[thread_IDs_index]];
    	tcbs[thread_IDs[thread_IDs_index]].prev_TCB = &tcbs[thread_IDs[thread_IDs_index]];
    	RunPt = &tcbs[0];
		NextRunPt = RunPt->next_TCB;
    	num_threads_active = 1;
    	first_thread = 0;
    }
    else{
    	if(num_threads_active < NUMTHREADS){
	    	num_threads_active += 1;
	    	TCB* temp = RunPt->next_TCB;
	    	RunPt->next_TCB = &tcbs[thread_IDs[thread_IDs_index]];
	    	temp->prev_TCB = &tcbs[thread_IDs[thread_IDs_index]];
	    	tcbs[thread_IDs[thread_IDs_index]].prev_TCB = RunPt;
	    	tcbs[thread_IDs[thread_IDs_index]].next_TCB = temp;
	  //   	if(thread_IDs[thread_IDs_index] == (num_threads_active-1)){
	  //   		tcbs[thread_IDs[thread_IDs_index]].next_TCB = &tcbs[0];
		 //    	tcbs[thread_IDs[thread_IDs_index]].prev_TCB = &tcbs[thread_IDs[thread_IDs_index]-1];
	  //   		tcbs[thread_IDs[thread_IDs_index]-1].next_TCB = &tcbs[thread_IDs[thread_IDs_index]];
	  //   		tcbs[0].prev_TCB = &tcbs[thread_IDs[thread_IDs_index]];
			// }
			// else{
			// 	tcbs[thread_IDs[thread_IDs_index]].next_TCB = &tcbs[thread_IDs[thread_IDs_index]+1];
		 //    	tcbs[thread_IDs[thread_IDs_index]].prev_TCB = &tcbs[thread_IDs[thread_IDs_index]-1];
	  //   		tcbs[thread_IDs[thread_IDs_index]-1].next_TCB = &tcbs[thread_IDs[thread_IDs_index]];
	  //   		tcbs[thread_IDs[thread_IDs_index]+1].prev_TCB = &tcbs[thread_IDs[thread_IDs_index]];
			// }
		}
		else{ //if num_threads_active = NUMTHREADS, no more threads allowed
			//OS_EnableInterrupts(status);
			EndCritical(status);
			return 0;
		}
    }
    SetInitialStack(thread_IDs[thread_IDs_index]);
    Stacks[thread_IDs[thread_IDs_index]][STACKSIZE-2] = (int32_t)(task); //PC
    tcbs[thread_IDs[thread_IDs_index]].Priority = priority;
    tcbs[thread_IDs[thread_IDs_index]].Sleep_state = 0;
    tcbs[thread_IDs[thread_IDs_index]].ID = thread_IDs[thread_IDs_index];
	thread_IDs_index += 1;
    EndCritical(status);
    return 1;
}

unsigned long lastPeriodic = 0;
//if the next TCB's sleep_state is not 0, the TCB will be skipped by the scheduler
void scheduler(void)
{
	TCB* holder = RunPt->next_TCB;
//	if(time - lastPeriodic > periodicThreadPeriod)
//	{
//		periodicTask();
//		lastPeriodic = time;
//	}
	while(holder->Sleep_state != 0)
	{
		holder = holder->next_TCB;
	}
	NextRunPt = holder;
	
}

void OS_Suspend(void) 
{
	NVIC_ST_CURRENT_R = 0;
	scheduler();
	NVIC_INT_CTRL_R = 0x10000000; // Trigger PendSV, handler is in OSasm.s
}

void OS_Init(void)
{
	OS_DisableInterrupts();
	PLL_Init(Bus80MHz);
	UART_Init();//enables interrupts
	ST7735_InitR(INITR_REDTAB);
	time = 0;
	//SysTick_Init();
	
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_CURRENT_R = 0;
	
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000;	// Systick has a priority of 6 ---     31 27 23 19 15 11 7 3
  																//  								   C  0  0  0  0  0  0 0

	NVIC_INT_CTRL_R = 0;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0xFF00FFFF) | 0x00E00000; // Give the PendSV a priority of 7    31 27 23 19 15 11 7 3 
}																 //                                    0  0  E  0  0  0  0 0

//Lab 2 part 2 prep
// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value)
{
	semaPt->Value = value;
}

//Lab 2 part 2 prep
// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt)
{
	OS_DisableInterrupts();
	while(semaPt->Value == 0){
		OS_EnableInterrupts();
		OS_Suspend();
		OS_DisableInterrupts();
	}
	semaPt->Value -= 1;
	OS_EnableInterrupts();
}

//Lab 2 part 2 prep
// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt)
{
	OS_DisableInterrupts();
	semaPt->Value += 1;
	OS_EnableInterrupts();
}

//Lab 2 part 2 prep
// ******** OS_bWait ************
// Lab2 spinlock, set to 0
//if the semaphore value is 0, OS will go to next thread
//if semaphore value is 1 or more, value is decremented and wait exits, allowing the thread to run
// there is one semaphore per resource that is used by multiple threads
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt)
{
	OS_DisableInterrupts();
	while(semaPt->Value == 0)
	{
		OS_EnableInterrupts(); 	
		OS_Suspend();
		OS_DisableInterrupts(); 
	}
	semaPt->Value = 0;
	OS_EnableInterrupts();
}

//Lab 2 part 2 prep
// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt)
{
	OS_DisableInterrupts();
	semaPt->Value = 1;
	OS_EnableInterrupts();
}


//where OS_AddThread was

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void)
{
	return RunPt->ID;
}

#define MAX_BACKGROUND 4
uint32_t periodic_count;
uint32_t background_count;
uint32_t periodic_reload;
BackgroundThread background[MAX_BACKGROUND];
void periodicHandler(void) {
	periodic_count+=TIMER3_TAILR_R;
	if(background_count > 0)
	{
		for(int i = 0; i < MAX_BACKGROUND; i++)
		{
			if(background[i].alive > 0 && background[i].period%(periodic_count - background[i].start_time) == 0) {
				background[i].task();
			}				
		}
	}
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, unsigned long priority)
{ 
//	OS_DisableInterrupts();
	//long sr = StartCritical(); 
  //OSTask = task;          // user function
  //function_priority = priority;
	/*
	int i = 0;
	while(background[i].alive)
	{
		i++;
	}
	if(i == MAX_BACKGROUND)
	{
		return 0;
	}
	background[i].alive = 1;
	background[i].period = period;
	background[i].start_time = periodic_count;
	background[i].task = task;
	background_count += 1;
	*/
  Timer2_Init(*task, period);
//	periodicThreadPeriod = period;
//	periodicTask = task;
  //EndCritical(sr);
//	OS_EnableInterrupts();
	return 1;
}
Sema4Type SW1,SW2;
void Switch_Init(void)
{
	SYSCTL_RCGCGPIO_R |= 0x20;		//activate clock on Port F
	int delay = 1;
	delay = 1;
	delay = 1;
	GPIO_PORTF_LOCK_R = 0X4C4F434B;		//unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x1F;		//allow changes to PF4-0
}

void SwitchTwo_Init(void)
{
	Switch_Init();
	OS_InitSemaphore(&SW2,0);
	GPIO_PORTF_DIR_R &= ~0x01;	//amke PF0 in 
	GPIO_PORTF_DEN_R |= 0x01;		//enable digital I/O on PF0
	GPIO_PORTF_PUR_R |= 0x01;		//pullup on PF1
	GPIO_PORTF_IS_R &= ~0x01;		//PF0 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x01;		//PF0 are rising 
	GPIO_PORTF_IEV_R &= ~0x01;
	GPIO_PORTF_ICR_R = 0x11;		//clear flags
	GPIO_PORTF_IM_R |= 0x01;		//arm interrupts on PF0
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00200000;	//priority 1
	NVIC_EN0_R = 0x40000000;		//enable interrrupt 40 in NVIC
}

void SwitchOne_Init(void)
{
	Switch_Init();
	OS_InitSemaphore(&SW1,0);
	GPIO_PORTF_DIR_R &= ~0x10;	//make PF4 in 
	GPIO_PORTF_DEN_R |= 0x10;		//enable digital I/O on PF0
	GPIO_PORTF_PUR_R |= 0x10;		//pullup on PF1
	GPIO_PORTF_IS_R &= ~0x10;		//PF0 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x10;		//PF0 are rising 
	GPIO_PORTF_IEV_R &= ~0x10;
	GPIO_PORTF_ICR_R = 0x11;		//clear flags
	GPIO_PORTF_IM_R |= 0x10;		//arm interrupts on PF0
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00200000;	//priority 1
	NVIC_EN0_R = 0x40000000;		//enable interrrupt 40 in NVIC
}

void (*SwOne_Task)(void);
void (*SwTwo_Task)(void);

void GPIOPortF_Handler(void)
{
	if(GPIO_PORTF_RIS_R&0x10)
	{
		GPIO_PORTF_ICR_R = 0x10; //acknowledge interrupt
		SwOne_Task();
	}
	if(GPIO_PORTF_RIS_R&0x01)
	{
		if(GPIO_PORTF_DATA_R|0x01)
		{
			GPIO_PORTF_ICR_R = 0x01;
			SwTwo_Task();
		}
	}
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), unsigned long priority)
{
	//OS_bWait(&SW1);
	SwOne_Task = task;
	SwitchOne_Init();
	//OS_bSignal(&SW1);
	return 1;
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), unsigned long priority)
{
	SwTwo_Task = task;
	SwitchTwo_Init();
	return 1;
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime)
{
	RunPt->Sleep_state = sleepTime;
	OS_Suspend();
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void)
{
	TCB* temp = RunPt->prev_TCB;
	temp->next_TCB = RunPt->next_TCB;
	temp = RunPt->next_TCB;
	temp->prev_TCB = RunPt->prev_TCB;
	thread_IDs_index -= 1;
	thread_IDs[thread_IDs_index] = RunPt->ID;
	// RunPt = RunPt->next_TCB;
	// NextRunPt = NextRunPt->next_TCB;
	num_threads_active -= 1;
	OS_Suspend();
}

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size)
{
	fifo_head = 0;
	fifo_tail = 0;
	OS_InitSemaphore(&fifo_sema,0);
	OS_InitSemaphore(&fifo_get_sema,1);
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data)
{
	if((fifo_head+1)%FIFOSIZE == fifo_tail)
	{
		return 0;
	}
	OS_Fifo[fifo_head] = data;
	fifo_head+=1;
	fifo_head%=FIFOSIZE;
	OS_Signal(&fifo_sema);
	return 1;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void)
{
	OS_Wait(&fifo_sema);
	OS_bWait(&fifo_get_sema);
	unsigned long ret = OS_Fifo[fifo_tail];
	fifo_tail+=1;
	fifo_tail%=FIFOSIZE;
	OS_bSignal(&fifo_get_sema);
	return ret;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void)
{
	if(fifo_head > fifo_tail) {
		return fifo_head - fifo_tail;
	}
	else if(fifo_head < fifo_tail) {
		return fifo_head + FIFOSIZE - fifo_tail;
	}
	return 0;
}

Sema4Type mailSema4Send,mailSema4Recieve;
unsigned long message;
// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void)
{
	OS_InitSemaphore(&mailSema4Send,1);
	OS_InitSemaphore(&mailSema4Recieve,0);
	message = 0;
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data)
{
	OS_bWait(&mailSema4Send);
	message = data;
	OS_bSignal(&mailSema4Recieve);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void)
{
	unsigned long ret;
	OS_bWait(&mailSema4Recieve);
	ret = message;
	OS_bSignal(&mailSema4Send);
	return ret;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295 -> biggest 32 bit #
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void)
{
	// OS_DisableInterrupts();
	uint32_t nvic_reload = NVIC_ST_RELOAD_R;
	uint32_t nvic_current = NVIC_ST_CURRENT_R;
	uint32_t current_clock = NVIC_ST_RELOAD_R - NVIC_ST_CURRENT_R;
	return time + current_clock;
	// OS_EnableInterrupts();
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop)
{
	return stop - start;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void)
{
	time = 0;
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void)
{
	return time/800000;
}

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(unsigned long theTimeSlice)
{
	NVIC_ST_RELOAD_R = theTimeSlice - 1;
	NVIC_ST_CTRL_R = 0x00000007;
	//RunPt = &tcbs[0];
	//NextRunPt = RunPt->next_TCB;
	StartOS();
}

void sleeper(void) 
{
	for(int i = 0; i < NUMTHREADS; i++)
	{
		if(tcbs[i].Sleep_state)
		{
			tcbs[i].Sleep_state-=1;
		}
	}
}


// Implements a preemptive scheduler
// Triggers a PendSV interrupt when the SysTick fires
void SysTick_Handler(void)
{
	// GPIO_PORTB_DATA_R ^= 0x20;	//Systick Heartbeat - PB5
	// GPIO_PORTB_DATA_R ^= 0x20;
	// time+=NVIC_ST_RELOAD_R;
	time += TIME_2MS-1;
	scheduler();
	sleeper();//every systick interrupt, all threads sleep_state will decrement by 1
	NVIC_INT_CTRL_R = 0x10000000; // Trigger PendSV, handler is in OSasm.s
	// GPIO_PORTB_DATA_R ^= 0x20;
}
