#include <stm32f10x.h>

namespace {  // anonymous******************************************

const char *ptr;
int count;

bool getC(char &c) {
  if (ptr) c = *ptr++;
  else return false;
  if (count) { if (!--count) ptr = 0; }
  else { if (!*ptr) ptr = 0; }
  return true;
}

} // anonymous*****************************************************

extern "C" void USART1_IRQHandler(void) {
  char c;
  if (USART1->SR & USART_SR_TXE) {
    if (getC(c)) USART1->DR = c;
    else USART1->CR1 &= ~USART_CR1_TXEIE;
  }
}

bool uart1_send(const char* p, int n) {
  if (ptr) return false;
  ptr = p; count = n;
  return (USART1->CR1 |= USART_CR1_TXEIE);  // ie return true
}

bool uart1_print(const char* p) { return uart1_send(p, 0); }

bool uart1_read(char &c) {
  if (!(USART1->SR & USART_SR_RXNE)) return false;
  c = USART1->DR; return true;
}
