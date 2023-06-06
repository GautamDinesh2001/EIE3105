#include <stm32f10x.h>
#include "os.h"

const char *ptr;
unsigned x, y;
int iflag;
char buttons, byte;


bool uart2_print(const char* p) {
  if (ptr) return false;
  ptr = p;
  return (USART2->CR1 |= USART_CR1_TXEIE);  // ie return true
}

void loop_rx_rmt(void) {
  static int flag;
  int f = flag ^ iflag;
  if (!f) return;
  if (f & 1) { flag ^= 1; on_buttons(buttons); } //
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
		  switch (c & 0xF0) //1111 0000 
			{
      case 0xF0: buttons = c; iflag ^= 1; break;// 1111 XXXX
      case 0x40: x = c & 0x0f | byte; iflag ^= 2; break; //0100 XXXX
      case 0x60: y = c & 0x0f | byte; iflag ^= 4; break; //0110 XXXX
      default: byte = c;
    }
  }
}
