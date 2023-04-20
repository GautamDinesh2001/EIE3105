#include <stm32f10x.h>

// LEDs: A6:A0, C13
// BUTTONs: [B13] [B6, B9, B8, B5]
// ADCs: A7-ADC12_IN7, B0-ADC12_IN8, B1-ADC12_IN9(batt)

void __attribute__((weak)) on_buttons(char) {}
//void poll(unsigned);    // write your poll(unsigned) in main.cpp

namespace { //anonymous*************************************

volatile unsigned tick;
short ADCs[3];
int buttons = 0x2360;

void loop(unsigned t) {
  static unsigned tick;
  if (tick == t) return;
  //poll(tick = t);
  if (t & 15) return;
  ADC1->CR2 |= ADC_CR2_ADON;     // ADC start convert
  int b = GPIOB->IDR & 0x2360;
  if (buttons == b) return;
  buttons = b;
  if (b & 0x2000) b |= 0x80;
  on_buttons((b >> 5) & 0x1f);
}

} //anonymous***********************************************

short getX(void) { return ADCs[1]; }
short getY(void) { return ADCs[0]; }
//short getV(void) { return (ADCs[2] << 1) / 19; }
short getV(void) { return ADCs[2]; }

void init(void) {
  SysTick_Config(72000);     // 1 ms
  RCC->APB1ENR = RCC_APB1ENR_USART3EN;
  RCC->APB2ENR = RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADC1EN | 
    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN;
  RCC->AHBENR = RCC_AHBENR_DMA1EN;
  GPIOA->CRL = 0x02222222;   // ADC12_IN7, LEDs
  GPIOA->CRH = 0x444444a4;   // TX1
  GPIOB->CRL = 0x48844400;   // B6, B5, ADC12_IN9, ADC12_IN8
  GPIOB->CRH = 0x44844a88;   // B13, TX3, B9, B8
  GPIOC->CRH = 0x44244444;   // LED
  GPIOB->BSRR = 0x2360;      // pull-up
  USART1->BRR = 625;         // 72000000/115200
  USART3->BRR = 312;         // 36000000/115200
  USART1->CR1 = USART3->CR1 = USART_CR1_TE | USART_CR1_RE;
  ADC1->SMPR2 = 0x1ff << 21; // SMP7=SMP8=SMP9=0b111 (239.5 cycles)
  ADC1->SQR1 = 2 << 20;      // sequence length = 3
  ADC1->SQR3 = 9<<10 | 8<<5 | 7;   // ch9, ch8, ch7
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 = ADC_CR2_DMA;
  DMA1_Channel1->CCR = DMA_CCR1_MSIZE_0 |
                       DMA_CCR1_PSIZE_0 |
                       DMA_CCR1_MINC |
                       DMA_CCR1_CIRC;
  DMA1_Channel1->CPAR = (int)(&ADC1->DR);
  DMA1_Channel1->CMAR = (int)ADCs;
  DMA1_Channel1->CNDTR = 3;
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_EnableIRQ(USART3_IRQn);
  USART1->CR1 |= USART_CR1_UE;
  USART3->CR1 |= USART_CR1_UE;
  DMA1_Channel1->CCR |= DMA_CCR1_EN;
}

bool uart1_send(char c) {
  if (USART1->SR & USART_SR_TXE) USART1->DR = c;
  else return false;
  return true;
}

bool uart3_send(char c) {
  if (USART3->SR & USART_SR_TXE) USART3->DR = c;
  else return false;
  return true;
}

void led(char c) {
  if (c & 1) GPIOC->BSRR = 0x2000; else GPIOC->BRR = 0x2000;
  int i = c >> 1 & 127;
  GPIOA->BSRR = i; GPIOA->BRR = ~i;
}

void wait(unsigned t) {
  if (!t) return loop(tick);
  t += tick;
  while (t != tick) loop(tick);
}

extern "C" void SysTick_Handler(void) { tick++; }
