#include <stm32f10x.h>
#include "stdio.h" //upd
#include "os.h"

void __attribute__((weak)) on_buttons(char) {}
void __attribute__((weak)) on_x(unsigned) {}
void __attribute__((weak)) on_y(unsigned) {}

namespace {

const char *ptr;
unsigned x, y;
int iflag;
char buttons, byte;

}

bool uart2_print(const char* p) {
  if (ptr) return false;
  ptr = p;
  return (USART2->CR1 |= USART_CR1_TXEIE);  // ie return true
}

void loop_remote(void) {
	static int flag;
  int f = flag ^ iflag;
  if (!f) return;
  if (f & 1) { flag ^= 1; on_buttons(buttons); }
  if (f & 2) { flag ^= 2; on_x(x); }
  if (f & 4) { flag ^= 4; on_y(y); }
}

extern "C" void USART2_IRQHandler(void) {
  if (USART2->SR & USART_SR_TXE) {
    if (ptr) {
      if (*ptr) USART2->DR = *ptr++; else ptr = 0;
    } else USART2->CR1 &= ~USART_CR1_TXEIE;
  }
  if (USART2->SR & USART_SR_RXNE) {
    char c = USART2->DR;
		
		/*
		char msg[2]= {c, c};
    if (c == 'u'|| c =='g')uart3_print(msg);
		*/
		
		/*
		switch (c & 0xc0) {
      case 0x40: buttons = c; iflag ^= 1; break;
      case 0x80: x = (c & 0x3f) << 6 | byte; iflag ^= 2; break;
      case 0xc0: y = (c & 0x3f) << 6 | byte; iflag ^= 4; break;
      default: byte = c;
    }
		*/
		
		//================================ UPDATE BY ALISHER 
		
		on_buttons(c);
		char msg[1]= {c};
		uart3_print(msg);
		
		//================================
		
		/*
		//added from other code
		switch (c & 0xF0) //1111 0000 
		{
			case 0xF0: buttons = c; iflag ^= 1; break;// 1111 XXXX
			case 0x40: x = c & 0x0f | byte; iflag ^= 2; break; //0100 XXXX
			case 0x60: y = c & 0x0f | byte; iflag ^= 4; break; //0110 XXXX
			default: byte = c;
    }
		*/
  }
}