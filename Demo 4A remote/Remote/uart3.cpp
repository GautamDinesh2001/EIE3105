#include <stm32f10x.h>

namespace { //anonymous*****************************************

const char *ptr;
int count;

bool getC(char &c) {
  if (ptr) c = *ptr++;
  else return false;
  if (count) { if (!--count) ptr = 0; }
  else { if (!*ptr) ptr = 0; }
  return true;
}

} //anonymous***************************************************

extern "C" void USART3_IRQHandler(void) {
  char c;
  if (USART3->SR & USART_SR_TXE) {
    if (getC(c)) USART3->DR = c;
    else USART3->CR1 &= ~USART_CR1_TXEIE;
  }
}

bool uart3_send(const char* p, int n) {
  if (ptr) return false;
  ptr = p; count = n;
  return (USART3->CR1 |= USART_CR1_TXEIE);  // ie return true
}

bool uart3_print(const char* p) { return uart3_send(p, 0); }

bool uart3_read(char &c) {
  if (!(USART3->SR & USART_SR_RXNE)) return false;
  c = USART3->DR; return true;
}
