#include "stm32f10x.h"                  // Device header
#include "os.h"
#include "stdio.h"
#include "misc.h"
#define auto_stop 0
int dist = 0;
char buffer[30];

int main()
{
	init();
	while(1)
	{
		loop_rx_rmt();
		
	
  }
}
