#include "stm32f10x.h"                  // Device header
#include "os.h"
#include "stdio.h"
#include "misc.h"

void uart1_send(const char* p, int n); // defined in uart1
void ADC_select(short X, short Y);

char msg[] = "0";
char msg_1[] = "^";
char msg_2[] = "v";
char msg_3[] = "<";
char msg_4[] = ">";
int top = 0;
int bottom = 0;
int left = 0;
int right = 0;
char buffer[50] = {'\0'};

int main(void)
{
 init();
 while(1)
 {
  ADC1->CR2 |= ADC_CR2_ADON; 
  
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) != 1)
  {
   uart1_send(msg,sizeof(msg));
   uart3_send(0xF0);
  }
  else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) != 1)
  {
   uart1_send(msg_1,sizeof(msg_1));
   uart3_send(0xF1); //1111 0001
  }
  else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) != 1)
  {
   uart1_send(msg_2,sizeof(msg_2));
   uart3_send(0xF2); //1111 0010
  }
  else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) != 1)
  {
   uart1_send(msg_3,sizeof(msg_3));
   uart3_send(0xF4); //1111 0100
  }
  else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) != 1)
  {
   uart1_send(msg_4,sizeof(msg_4));
   uart3_send(0xF8); //1111 1000
  }
  
  ADC_select(getX(), getY());
  sprintf(buffer, "V=%d X=%d Y=%d\r\n", getV(), getX(), getY()); // for debugging
  uart1_send(buffer, sizeof(buffer)); 
 }
}

void ADC_select(short X, short Y)
{
	//1-forward
	//2-backward
	//3-left
	//4-right
	//convert 4096 to 16, 12->4 / 2^8 = 256
	char temp_x = X / 256 + 0x00;
	char temp_y = Y / 256 + 0x00;
	
	if (X > 3000)
 {
  uart3_send(0x40 + temp_x); //0100 XXXX 
 }
 else if (X < 1000)
 {
  uart3_send(0x40 + temp_x);
 }
 else if (Y > 3000)
 {
  uart3_send(0x60 + temp_y); //0110 XXXX 
 }
 else if (Y < 1000)
 {
  uart3_send(0x60 + temp_y);
 }
 else
 {
  uart3_send(0x70); //0111 0000
 }
}
