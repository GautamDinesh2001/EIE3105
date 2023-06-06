#include <stm32f10x.h>
#include "os.h"

static const char *ptr;

extern "C" void USART3_IRQHandler(void) {
  if (USART3->SR & USART_SR_TXE) {
    if (ptr) {
      if (*ptr) USART3->DR = *ptr++; else ptr = 0;
    } else USART3->CR1 &= ~USART_CR1_TXEIE;
  }
}

bool uart3_print(const char* p) {
  if (ptr) return false;
  ptr = p;
  return (USART3->CR1 |= USART_CR1_TXEIE);  // ie return true
}

bool uart3_read(char &c) {
  if (!(USART3->SR & USART_SR_RXNE)) return false;
  c = USART3->DR; return true;
}