	//*****************************************************************************
//
// Lab4.c - user programs, File system, stream data onto disk
//*****************************************************************************

// Jonathan W. Valvano 3/7/17, valvano@mail.utexas.edu
// EE445M/EE380L.12
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file to do Lab 4
// as long as the basic functionality is simular
// 1) runs on your Lab 2 or Lab 3
// 2) implements your own eFile.c system with no code pasted in from other sources
// 3) streams real-time data from robot onto disk
// 4) supports multiple file reads/writes
// 5) has an interpreter that demonstrates features
// 6) interactive with UART input, and switch input

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task 
// PF4 is SW1 button input

// Analog inputs
// PE3 sequencer 3, channel 3, J8/PE0, sampling in DAS(), software start
// PE0 timer-triggered sampling, channel 0, J5/PE3, 50 Hz, processed by Producer
//******Sensor Board I/O*******************
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) connected to PB0
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// HC-SR04 Ultrasonic Range Finder 
// J9X  Trigger0 to PB7 output (10us pulse)
// J9X  Echo0    to PB6 T0CCP0
// J10X Trigger1 to PB5 output (10us pulse)
// J10X Echo1    to PB4 T1CCP0
// J11X Trigger2 to PB3 output (10us pulse)
// J11X Echo2    to PB2 T3CCP0
// J12X Trigger3 to PC5 output (10us pulse)
// J12X Echo3    to PF4 T2CCP0

// Ping))) Ultrasonic Range Finder 
// J9Y  Trigger/Echo0 to PB6 T0CCP0
// J10Y Trigger/Echo1 to PB4 T1CCP0
// J11Y Trigger/Echo2 to PB2 T3CCP0
// J12Y Trigger/Echo3 to PF4 T2CCP0

// IR distance sensors
// J5/A0/PE3
// J6/A1/PE2
// J7/A2/PE1
// J8/A3/PE0  

// ESP8266
// PB1 Reset
// PD6 Uart Rx <- Tx ESP8266
// PD7 Uart Tx -> Rx ESP8266

// Free pins (debugging)
// PF3, PF2, PF1 (color LED)
// PD3, PD2, PD1, PD0, PC4

#include "OS.h"
#include "../inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include "eDisk.h"
#include "eFile.h"
#include "SysTick.h"
#include "PLL.h"
#include "Timer2.h"
#include "Timer3.h"

#include <string.h> 
#include <stdio.h>
#include <stdint.h>

//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);

#define PF0  (*((volatile unsigned long *)0x40025004))
#define PF1  (*((volatile unsigned long *)0x40025008))
#define PF2  (*((volatile unsigned long *)0x40025010))
#define PF3  (*((volatile unsigned long *)0x40025020))
#define PF4  (*((volatile unsigned long *)0x40025040))
  
#define PD0  (*((volatile unsigned long *)0x40007004))
#define PD1  (*((volatile unsigned long *)0x40007008))
#define PD2  (*((volatile unsigned long *)0x40007010))
#define PD3  (*((volatile unsigned long *)0x40007020))



void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_PRGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;   // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;     // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;      // disable analog functionality on PD
}  
unsigned long NumCreated;   // number of foreground threads created
unsigned long NumSamples;   // incremented every sample
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished

int Running;                // true while robot is running

#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units
long x[64],y[64];           // input and output arrays for FFT
Sema4Type doFFT;            // set every 64 samples by DAS

long median(long u1,long u2,long u3){ 
long result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else 
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}
//------------ADC2millimeter------------
// convert 12-bit ADC to distance in 1mm
// it is known the expected range is 100 to 800 mm
// Input:  adcSample 0 to 4095 
// Output: distance in 1mm
long ADC2millimeter(long adcSample){
  if(adcSample<494) return 799; // maximum distance 80cm
  return (268130/(adcSample-159));  
}
long Distance3;     // distance in mm on IR3
uint32_t Index3;    // counts to 64 samples
long x1,x2,x3;
void DAS(void){ 
long output;  
  PD0 ^= 0x01;
  x3=x2; x2= x1;  // MACQ
  x1 = ADC_In();  // channel set when calling ADC_Init
  PD0 ^= 0x01;
  if(Index3<64){
    output = median(x1,x2,x3); // 3-wide median filter
    Distance3 = ADC2millimeter(output);
    FilterWork++;        // calculation finished
    x[Index3] = Distance3;
    Index3++;
    if(Index3==64){
      OS_Signal(&doFFT);
    }
  }
  PD0 ^= 0x01;
}

void DSP(void){ 
unsigned long DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
  OS_InitSemaphore(&doFFT,0); 
  while(1) { 
    OS_Wait(&doFFT); // wait for 64 samples
    PD2 = 0x04;
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    PD2 = 0x00;
    Index3 = 0; // take another buffer
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    ST7735_Message(1,0,"IR3 (mm) =",DCcomponent);    
  }
}
char Name[8]="robot0";
//******** Robot *************** 
// foreground thread, accepts data from producer
// inputs:  none
// outputs: none
void Robot(void){   
unsigned long data;      // ADC sample, 0 to 1023
unsigned long voltage;   // in mV,      0 to 3300
unsigned long distance;  // in mm,      100 to 800
unsigned long time;      // in 10msec,  0 to 1000 
  OS_ClearMsTime();    
  DataLost = 0;          // new run with no lost data 
  OS_Fifo_Init(256);
  // printf("Robot running...");<-- uncomment
  // eFile_RedirectToFile(Name); // robot0, robot1,...,robot7  <-- uncomment
  // printf("time(sec)\tdata(volts)\tdistance(mm)\n\r");<-- uncomment
  do{
    PIDWork++;    // performance measurement
    time=OS_MsTime();            // 10ms resolution in this OS
    data = OS_Fifo_Get();        // 1000 Hz sampling get from producer
    voltage = (300*data)/1024;   // in mV
    distance = ADC2millimeter(data);
    // printf("%0u.%02u\t%0u.%03u \t%5u\n\r",time/100,time%100,voltage/1000,voltage%1000,distance);<-- uncomment
  }
  while(time < 200);       // change this to mean 2 seconds
  // eFile_EndRedirectToFile(); <--uncomment
  ST7735_Message(0,1,"IR0 (mm) =",distance); 
  // printf("done.\n\r");<-- uncomment
  Name[5] = (Name[5]+1)&0x07; // 0 to 7
  Running = 0;             // robot no longer running
  OS_Kill();
}
  
//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two robot threads
    NumCreated += OS_AddThread(&Robot,128,1);  // start a 2 second run
  }
}
//************SW2Push*************
// Called when SW2 Button pushed
// background threads execute once and return
void SW2Push(void){

}



//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 1 kHz, started by your ADC_Collect
// The timer triggers the ADC, creating the 1 kHz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 10-bit sample 
// sends data to the Robot, runs periodically at 1 kHz
// inputs:  none
// outputs: none
void Producer(unsigned long data){  
  if(Running){
    if(OS_Fifo_Put(data)){     // send to Robot
      NumSamples++;
    } else{ 
      DataLost++;
    } 
  }
}
 
//******** IdleTask  *************** 
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
unsigned long Idlecount=0;
void IdleTask(void){ 
  while(1) { 
    Idlecount++;        // debugging 
  }
}

// uint16_t strcmp(char* str1, char* str2)
// {
//     uint32_t i = 0;
//     while(str1[i]!=0 && str2[i]!=0)
//     {
//         if(str1[i] != str2[i])
//         {
//             return -1;
//         }
//         i++;
//     }
//     if(str1[i] == str2[i])
//     {
//         return 0;
//     }
//     return 1;

// }


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


//******** Interpreter **************
// your intepreter from Lab 4 
// foreground thread, accepts input from UART port, outputs to UART port
// inputs:  none
// outputs: none
// add the following commands, remove commands that do not make sense anymore
// 1) format 
// 2) directory 
// 3) print file
// 4) delete file
// execute   eFile_Init();  after periodic interrupts have started
// extern void Interpreter(void); 
void Interpreter(void){    // just a prototype, link to your interpreter
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 
  GPIO_PORTB_DATA_R ^= 0x4;//PB2
  char userInput[20];
  while(1){
    GPIO_PORTB_DATA_R ^= 0x4;//PB2
    UART_OutString("--> ");
    UART_InString(userInput,20);
    UART_OutString("\n\r");
    if(strcmp(userInput,"ADC_Read") == 0)
    {
      UART_OutString("ADC Value:");
      char val[10];
      //itoa(ADC_In(), val, 10); 
      itoa(1, val, 10); 
      UART_OutString(val);
    }
//    else if(strcmp(userInput,"ADC_Collect") == 0){
//      UART_OutString("Enter Period: ");
//      uint32_t user_period = UART_InUDec();
//      UART_OutString("\r\n");
//      UART_OutString("Number of Samples: ");
//      uint32_t user_num_samples = UART_InUDec();
//      uint16_t userDataBuffer[user_num_samples];
//      ADC_Collect(0, user_period, userDataBuffer, user_num_samples);
//    }
    else if(strcmp(userInput,"Hello") == 0)
    {
      UART_OutString("Hi!");
    }
    else if(strcmp(userInput, "Time") == 0)
    {
      uint32_t time = OS_MsTime();
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
      ST7735_Message(screen,line,userInput,0);      
    }
    else if(strcmp(userInput, "Clear Time") == 0)
    {
      OS_ClearMsTime();
      UART_OutString("Time reset to 0."); 
    }
    else if(strcmp(userInput, "Clear Screen") == 0)
    {
      ST7735_FillScreen(ST7735_BLACK); 
      ST7735_DivideScreen();
      UART_OutString("Screen cleared");
    }
    else if(strcmp(userInput, "Print time") == 0)
    {
      UART_OutUDec(OS_Time());
    }
    else if(strcmp(userInput, "Print jitter") == 0)
    {
      // UART_OutUDec(MaxJitter);
    }
    else if(strcmp(userInput, "Threads created") == 0)
    {
      UART_OutUDec(NumCreated);
    }
    else if(strcmp(userInput,"NumSamples") == 0)
    {
      UART_OutUDec(NumSamples);
    }
    else if(strcmp(userInput, "DataLost") == 0)
    {
      UART_OutUDec(DataLost);
    }
    else if(strcmp(userInput, "FilterWork") == 0)
    {
      UART_OutUDec(FilterWork);
    }
    else if(strcmp(userInput, "Dump thread") == 0)
    {
      dumpLog();
    }
    else if(strcmp(userInput, "Dump periodic1") == 0)
    {
      dumpPeriodic1();
    }
    else if(strcmp(userInput, "Dump periodic2") == 0)
    {
      dumpPeriodic2();
    }
    else if(strcmp(userInput,"Reset thread log") == 0)
    {
      resetLog();
    }
    else if(strcmp(userInput,"Reset periodic1 log") == 0)
    {
      resetPeriodic1();
    }
    else if(strcmp(userInput,"Reset periodic2 log") == 0)
    {
      resetPeriodic2();
    }
    else 
    {
      UART_OutString("That is not a recognized command.");
    }
    UART_OutString("\n\r");
  }
}

//*******************lab 4 main **********
int realmain(void){        // lab 4 real main
  OS_Init();           // initialize, disable interrupts
  Running = 0;         // robot not running
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  PortD_Init();  // user debugging profile
  
//********initialize communication channels
  OS_Fifo_Init(256);    
  ADC_Collect(0, 50, &Producer); // start ADC sampling, channel 0, PE3, 50 Hz
  // ADC_Init(3);  // sequencer 3, channel 3, PE0, sampling in DAS()
  ADC_Open(3);  // sequencer 3, channel 3, PE0, sampling in DAS()
  OS_AddPeriodicThread(&DAS,10*TIME_1MS,1); // 100Hz real time sampling of PE0

//*******attach background tasks***********
  OS_AddSW1Task(&SW1Push,2);    // PF4, SW1
  OS_AddSW2Task(&SW2Push,3);   // PF0
  OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,5);

  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&DSP,128,1); 
  NumCreated += OS_AddThread(&IdleTask,128,7);  // runs when nothing useful to do
 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 
//*****************test project 0*************************
// This is the simplest configuration, 
// Just see if you can import your OS
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops
void Thread1(void){
  Count1 = 0;          
  for(;;){
    PD0 ^= 0x01;       // heartbeat
    Count1++;
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    PD1 ^= 0x02;       // heartbeat
    Count2++;
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    PD2 ^= 0x04;       // heartbeat
    Count3++;
  }
}

int Testmain0(void){  // Testmain0
  OS_Init();          // initialize, disable interrupts
  PortD_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,2); 
  NumCreated += OS_AddThread(&Thread3,128,3); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
//*****************test project 1*************************
unsigned char buffer[512];
#define MAXBLOCKS 100
void diskError(char* errtype, unsigned long n){
  UART_OutString(errtype);
  // printf(" disk error %u",n);
  UART_OutString(" disk error ");
  UART_OutUDec(n);
  UART_OutString("\t\n");
  OS_Kill();
}
void TestDisk(void){  DSTATUS result;  unsigned short block;  int i; unsigned long n;//DSTATUS is an unsigned char
  // simple test of eDisk
  // ST7735_OutString(0, 1, "eDisk test      ", ST7735_WHITE);
  // ST7735_Message (0, 1, "eDisk test      ", 0);<-- uncomment
  // printf("\n\rEE445M/EE380L, Lab 4 eDisk test\n\r");
  UART_OutString("\n\rEE445M/EE380L, Lab 4 eDisk test\n\r");
  result = eDisk_Init(0);  // initialize disk
  if(result) diskError("eDisk_Init",result);
  // printf("Writing blocks\n\r");
  UART_OutString("Writing blocks\n\r");
  n = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      buffer[i] = 0xFF&n;        
    }
    PD3 = 0x08;     // PD3 high for 100 block writes
    if(eDisk_WriteBlock(buffer,block))diskError("eDisk_WriteBlock",block); // save to disk
    PD3 = 0x00;     
  }  
  // printf("Reading blocks\n\r");
  UART_OutString("Reading blocks\n\r");
  n = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    PD2 = 0x04;     // PF2 high for one block read
    if(eDisk_ReadBlock(buffer,block))diskError("eDisk_ReadBlock",block); // read from disk
    PD2 = 0x00;
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      if(buffer[i] != (0xFF&n)){
        // printf("Read data not correct, block=%u, i=%u, expected %u, read %u\n\r",block,i,(0xFF&n),buffer[i]);
        UART_OutString("Read data not correct, block=");
        UART_OutUDec(block);
        UART_OutString(", i=");
        UART_OutUDec(i);
        UART_OutString(", expected ");
        UART_OutUDec((0xFF&n));
        UART_OutString(", read ");
        UART_OutUDec(buffer[i]);
        UART_OutString("\n\r");
        OS_Kill();
      }      
    }
  }  
  // printf("Successful test of %u blocks\n\r",MAXBLOCKS);
  // UART_OutString("Successful test of %u blocks\n\r",MAXBLOCKS);
  UART_OutString("Successful test of ");
  UART_OutUDec(MAXBLOCKS);
  UART_OutString(" blocks\n\r");
  // ST7735_OutString(0, 1, "eDisk successful", ST7735_YELLOW);
  //ST7735_Message(0, 1, "eDisk successful", 0);
  Running=0; // launch again
  OS_Kill();
}
void RunTest(void){
  NumCreated += OS_AddThread(&TestDisk,128,1);  
}
//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push1(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&TestDisk,128,1);  // test eDisk
  }
}
//******************* test main1 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread
int testmain1(void){   // testmain1
  OS_Init();           // initialize, disable interrupts
  PortD_Init();
//*******attach background tasks***********
  OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
  OS_AddSW1Task(&SW1Push1,2);
  
  NumCreated = 0 ;
  Running = 1; 
// create initial foreground threads
  NumCreated += OS_AddThread(&TestDisk,128,1);  
  NumCreated += OS_AddThread(&IdleTask,128,3); 
  OS_AddSW1Task(&SW1Push1,2);    // PF4, SW1
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

//*****************test project 2*************************   uncomment

 void TestFile(void){   int i; char data; 
   printf("\n\rEE445M/EE380L, Lab 4 eFile test\n\r");
   // ST7735_OutString(0, 1, "eFile test      ", ST7735_WHITE);
   //ST7735_Message(0, 1, "eFile test      ", 0);
   // simple test of eFile
   if(eFile_Init())              diskError("eFile_Init",0); 
   if(eFile_Format())            diskError("eFile_Format",0); 
   eFile_Directory(&UART_OutChar);
   if(eFile_Create("file1"))     diskError("eFile_Create",0);
   if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
   for(i=0;i<1000;i++){
     if(eFile_Write('a'+i%26))   diskError("eFile_Write",i);
     if(i%52==51){
       if(eFile_Write('\n'))     diskError("eFile_Write",i);  
       if(eFile_Write('\r'))     diskError("eFile_Write",i);
     }
   }
   if(eFile_WClose())            diskError("eFile_WClose",0);
   eFile_Directory(&UART_OutChar);
   if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
   for(i=0;i<1000;i++){
     if(eFile_ReadNext(&data))   diskError("eFile_ReadNext",eFile_ReadNext(&data));
     UART_OutChar(data);
   }
   if(eFile_Delete("file1"))     diskError("eFile_Delete",0);
	 printf("\n\r");
	 printf("Hello");
	 printf("\n\r");
   eFile_Directory(&UART_OutChar);
   if(eFile_Close())             diskError("eFile_Close",0);
   //printf("Successful test of creating a file\n\r");
   // ST7735_OutString(0, 1, "eFile successful", ST7735_YELLOW);
   //ST7735_Message(0, 1, "eFile successful", 0);
   Running=0; // launch again
   OS_Kill();
 }
 
  void TestFile2(void){   int i; char data; 
   printf("\n\rEE445M/EE380L, Lab 4 eFile test AKA Jake and Michaels Awesome file system\n\r");
   // ST7735_OutString(0, 1, "eFile test      ", ST7735_WHITE);
   //ST7735_Message(0, 1, "eFile test      ", 0);
   // simple test of eFile
   if(eFile_Init())              diskError("eFile_Init",0); 
   if(eFile_Format())            diskError("eFile_Format",0); 
   eFile_Directory(&UART_OutChar);
   if(eFile_Create("file1"))     diskError("eFile_Create",0);
	 if(eFile_Create("jake")) diskError("eFile_Create Jake",42);
	 if(eFile_Create("mike")) diskError("eFile_Create Michael",43);
	 eFile_Directory(&UART_OutChar);
   if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
   for(i=0;i<1000;i++){
     if(eFile_Write('a'+i%26))   diskError("eFile_Write",i);
     if(i%52==51){
       if(eFile_Write('\n'))     diskError("eFile_Write",i);  
       if(eFile_Write('\r'))     diskError("eFile_Write",i);
     }
   }
   if(eFile_WClose())            diskError("eFile_WClose",0);
	 if(eFile_WOpen("jake")) diskError("Jake",3);
	 eFile_Write('j');
	 if(eFile_WClose()) diskError("Jake close",4);
	 if(eFile_WOpen("mike")) diskError("mike open",3);
	 eFile_Write('m');
	 if(eFile_WClose()) diskError("mike close",4);
	 
   eFile_Directory(&UART_OutChar);
   if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
   for(i=0;i<1000;i++){
     if(eFile_ReadNext(&data))   diskError("eFile_ReadNext",eFile_ReadNext(&data));
     UART_OutChar(data);
   }
	 eFile_RClose();
	 if(eFile_ROpen("jake"))      diskError("eFile_ROpen jake",0);
	 if(eFile_ReadNext(&data))   diskError("eFile_ReadNext jake",eFile_ReadNext(&data));
   UART_OutChar(data);
	 eFile_RClose();
   if(eFile_ROpen("mike"))      diskError("eFile_ROpen mike",0);
	 if(eFile_ReadNext(&data))   diskError("eFile_ReadNext mike",eFile_ReadNext(&data));
   UART_OutChar(data);
	 eFile_RClose();
   if(eFile_Delete("file1"))     diskError("eFile_Delete",0);
	 printf("\n\r");
   eFile_Directory(&UART_OutChar);
   if(eFile_Close())             diskError("eFile_Close",0);
   //printf("Successful test of creating a file\n\r");
   // ST7735_OutString(0, 1, "eFile successful", ST7735_YELLOW);
   //ST7735_Message(0, 1, "eFile successful", 0);
   Running=0; // launch again
   OS_Kill();
 }
 //************SW1Push2*************
 // Called when SW1 Button pushed
 // background threads execute once and return
 void SW1Push2(void){
   if(Running==0){
     Running = 1;  // prevents you from starting two test threads
     NumCreated += OS_AddThread(&TestFile,128,1);  // test eFile
   }
 }
 
 
 //******************* test main2 **********
 // SYSTICK interrupts, period established by OS_Launch
 // Timer interrupts, period established by first call to OS_AddPeriodicThread
 int main(void){     //testmain2
   OS_Init();           // initialize, disable interrupts
   PortD_Init();
   Running = 1; 

 //*******attach background tasks***********
   OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
   OS_AddSW1Task(&SW1Push1,2);    // PF4, SW1
   OS_AddSW2Task(&SW1Push2,2);    // PF0, SW2
   NumCreated = 0 ;
 // create initial foreground threads
   NumCreated += OS_AddThread(&TestFile2,128,1);  
   NumCreated += OS_AddThread(&IdleTask,128,3); 
 
   OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
   return 0;               // this never executes
 }
