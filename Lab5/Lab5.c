// Lab2.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3 Testmain4  and main
// Lab3: Testmain5 Testmain6, Testmain7, and main (with SW2)

// Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
// EE445M/EE380L.12
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task, samples PD3
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PD3 Ain3 sampled at 2k, sequencer 3, by DAS software start in ISR
// PD2 Ain5 sampled at 250Hz, sequencer 0, by Producer, timer tigger

//#include <string.h> 
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "OS.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include "SysTick.h"
#include "PLL.h"
#include "Interpreter.h"
#include "diskio.h"
#include "ff.h"

static FATFS g_sFatFs;
FIL Handle,Handle2;
FRESULT MountFresult;
FRESULT Fresult;
unsigned char buffer[512];
#define MAXBLOCKS 100
Sema4Type Heap_Sema;

// #define Lab2 1
// #define Lab3 0
//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
#define FS 400              // producer/consumer sampling
#define RUNLENGTH (20*FS)   // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US   // DAS 2kHz sampling period in system time units
long x[64],y[64];           // input and output arrays for FFT

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
// #if Lab2
// long MaxJitter = 0;             // largest time jitter between interrupts in usec
// #define JITTERSIZE 64
// unsigned long const JitterSize=JITTERSIZE;
// unsigned long JitterHistogram[JITTERSIZE]={0,};
// #endif
unsigned long TotalWithI1;
unsigned short MaxWithI1;

#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))

void PortE_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x10;       // activate port E
  while((SYSCTL_RCGCGPIO_R&0x10)==0){};      
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
}

void PortF_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x20;       // activate port E
  while((SYSCTL_RCGCGPIO_R&0x20)==0){};      
  GPIO_PORTF_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTF_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTF_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTF_PCTL_R = ~0x0000FFFF;
  GPIO_PORTF_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
}

void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x8;       // activate port D
  while((SYSCTL_RCGCGPIO_R&0x8)==0){};      
  GPIO_PORTD_DIR_R |= 0x4F;    // make PD7-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x4F;   // disable alt funct on PD7-0
  GPIO_PORTD_DEN_R |= 0x4F;     // enable digital I/O on PD7-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x4F;;      // disable analog functionality on PF
}

void PortB_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x2;       // activate port B
  while((SYSCTL_RCGCGPIO_R&0x2)==0){};      
  GPIO_PORTB_DIR_R |= 0xFF;    // make PB3-0 output heartbeats
  GPIO_PORTB_AFSEL_R &= ~0xFF;   // disable alt funct on PB3-0
  GPIO_PORTB_DEN_R |= 0xFF;     // enable digital I/O on PB3-0
  GPIO_PORTB_PCTL_R = ~0x0000FFFF;
  GPIO_PORTB_AMSEL_R &= ~0xFF;;      // disable analog functionality on PF
}
Sema4Type button_sema;
//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
	GPIO_PORTB_DATA_R ^= 0x1;//PB0
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

void Idle(void){ 
	while(1)
	{
		GPIO_PORTF_DATA_R ^= 0x02;//toggle PF1
	}
}


const char ELFFilename[] = "Proc.axf";
const char helloFilename[] = "hello.txt";
void memory_test(void){
  // PLL_Init(Bus80MHz);    // 80 MHz
  // UART_Init();
  UINT successfulreads, successfulwrites;
  uint8_t c, x, y;
  // EnableInterrupts();

  MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    UART_OutString("f_mount error\r\n");
    while(1){};
  }
  // open the file to be read
  Fresult = f_open(&Handle, helloFilename, FA_READ);
  if(Fresult == FR_OK){
    UART_OutString("Opened ");
    UART_OutString((char *)helloFilename);
    UART_OutString("\r\n");
    // get a character in 'c' and the number of successful reads in 'successfulreads'
    Fresult = f_read(&Handle, &c, 1, &successfulreads);
    x = 0;                              // start in the first column
    y = 10;                             // start in the second row
    while((Fresult == FR_OK) && (successfulreads == 1) && (y <= 130)){
      if(c == '\n'){
        x = 0;                          // go to the first column (this seems implied)
        y = y + 10;                     // go to the next row
      } else if(c == '\r'){
        x = 0;                          // go to the first column
      } else{                           // the character is printable, so print it
        UART_OutChar(c);
        x = x + 6;                      // go to the next column
        if(x > 122){                    // reached the right edge of the screen
          x = 0;                        // go to the first column
          y = y + 10;                   // go to the next row
        }
      }
      // get the next character in 'c'
      Fresult = f_read(&Handle, &c, 1, &successfulreads);
    }
    UART_OutString("\r\n");
    // close the file
    Fresult = f_close(&Handle);
   } else{
    // print the error code
    UART_OutString("Error          (  )");
    UART_OutString((char *)helloFilename);
    UART_OutString("\r\n");
  }
  OS_Kill();


}
int main(void){     //realmain
  OS_Init();           // initialize PLL, UART, disable interrupts
  //PortE_Init();
  PortB_Init();
  PortF_Init();
  PCB_TCB_Init();
  
  MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    UART_OutString("f_mount error\r\n");
  }
  // Fresult = f_open(&Handle, ELFFilename, FA_READ);
  // if(Fresult == FR_OK){
  //   UART_OutString("Opened ");
  //   UART_OutString((char *)ELFFilename);
  //   UART_OutString("\r\n");
  // }

  // OS_AddSW1Task(&SW1Push,2);
  // OS_AddSW2Task(&SW2Push,2);  
  NumCreated = 0 ;
// create initial foreground threads
  
//OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority)
  // NumCreated += OS_AddThread(&memory_test,128,2);
  NumCreated += OS_AddThread(&Interpreter,128,2);
  NumCreated += OS_AddThread(&Idle,128,7);  // Lab 3, make this lowest priority
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

