#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "OS.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include "SysTick.h"
#include "PLL.h"
#include "loader.h"

extern Sema4Type Heap_Sema;

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

static const ELFSymbol_t symtab[] = {
    { "ST7735_Message", ST7735_Message }
};

void LoadProgram() {
    ELFEnv_t env = { symtab, 1 };
    if (exec_elf("Proc.axf", &env) != -1) {
        UART_OutString("ELF loaded successfully\r\n");
    }
    else{
        UART_OutString("ELF load error\r\n");
    }
}
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
//		else if(strcmp(userInput,"ADC_Collect") == 0){
//			UART_OutString("Enter Period: ");
//			uint32_t user_period = UART_InUDec();
//			UART_OutString("\r\n");
//			UART_OutString("Number of Samples: ");
//			uint32_t user_num_samples = UART_InUDec();
//			uint16_t userDataBuffer[user_num_samples];
//			ADC_Collect(0, user_period, userDataBuffer, user_num_samples);
//		}
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
		else if(strcmp(userInput, "e") == 0)
		{
			// exec_elf("Proc.axf", &env);//<-- calls OS_AddProcess
            OS_bWait(&Heap_Sema);
            LoadProgram();
            OS_bSignal(&Heap_Sema);
		}
		// else if(strcmp(userInput, "Print jitter") == 0)
		// {
		// 	UART_OutUDec(MaxJitter);
		// }
		// else if(strcmp(userInput, "Threads created") == 0)
		// {
		// 	UART_OutUDec(NumCreated);
		// }
		// else if(strcmp(userInput,"NumSamples") == 0)
		// {
		// 	UART_OutUDec(NumSamples);
		// }
		// else if(strcmp(userInput, "DataLost") == 0)
		// {
		// 	UART_OutUDec(DataLost);
		// }
		// else if(strcmp(userInput, "FilterWork") == 0)
		// {
		// 	UART_OutUDec(FilterWork);
		// }
		else 
		{
			UART_OutString("That is not a recognized command.");
		}
		UART_OutString("\n\r");
	}
}
