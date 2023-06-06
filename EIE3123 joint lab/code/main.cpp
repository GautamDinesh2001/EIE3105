#include <stm32f10x.h>
#include "stdio.h"
#include "os.h"
#include "wheels.h"

char msg[50];
int dist; 
bool input;
int reading;
int prevreading = 0;
int main()
{
	init();
	while(1)
		{
				
			
			  controller();
			  
	}

/*void on_button(bool b) {
  static int state;
  if (b) {
						dirLeft(true);
		        dirRight(true);
		        pwmLeft(12000);
		        pwmRight(12000);
						while(1)
						{	sprintf(msg, "L_COUNT:%d R_COUNT:%d\r\n", getLeft(), getRight());
		         uart2_print (msg);
						}
  }*/
}
