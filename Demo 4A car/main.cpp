#include "stm32f10x.h"                  // Device header
#include "os.h"
#include "stdio.h"
#include "misc.h"
#include "stdlib.h"
#define auto_stop 0
int dist = 0;
char buffer[30];
unsigned char reading;
char temp_msg[50];
int Counter = 0;
char bits[10];
int stage = 1;
int newreading = 0;
int nextstage;
int prevbits[10];
void DelayMs(uint32_t ms);
static __IO uint32_t msTicks;
int main()
{
	init();
	while(1)
	{
		
		
		
		reading = getDistance();
		//LED8(reading);
		
		
		/*sprintf (temp_msg, "%d\r\n", reading);
	  uart3_print (temp_msg);
			wait(300);*/
		 if(reading <= 105)
		 {
			 pwmLeft(0);
			 pwmRight(0);
			 dirLeft(true);
			 dirRight(true);
		 }
		 else  
		 {
			 pwmLeft(12000);
			 pwmRight(12000);
			 dirLeft(true);
			 dirRight(true);
		 }
	  	
		 }
	 }

 