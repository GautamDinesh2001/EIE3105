#include <stm32f10x.h>
#include "stdio.h"
#include "os.h"

void __attribute__((weak)) on_button(bool) {}
void poll(unsigned), pwms(unsigned, unsigned, char), sr04(unsigned);
char getPath(void);
void loop_remote(void);

//added 
int speed = 20000;
int angular_speed = 15000;
char spiValue;
char getSpiVal() {return spiValue;}
	

int getSpeed(){return speed;}
int getAngularSpeed(){return angular_speed;}

void Delay (int);

unsigned spi_cnt, left_cnt, right_cnt; // Moved from anonymous

int getRight(){return left_cnt;}
int getLeft() {return right_cnt;}


namespace { //anonymous************************************************
	
volatile unsigned tick;

int button = 0x1000;
char path, leds = 0x80;
	

void loop(unsigned t) {
  static unsigned tick;
  loop_remote();
  if (tick == t) return;
  pwms(left_cnt, right_cnt, getPath());
  //poll(tick = t);
  /*if (t & 7) return;
  ADC1->CR2 |= ADC_CR2_ADON;
  int b = GPIOA->IDR & 0x1000;
  if (button == b) return;
  on_button((button = b));
	*/
}

char flip(char c, char m) {
  char cm = c & m;
  return cm ? (cm == m ? c : c ^ m) : c;
}

} //anonymous**********************************************************

void init(void) {
  RCC->APB1ENR = RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN |
                 RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM2EN;
  RCC->APB2ENR = RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM1EN |
                 RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN |
                 RCC_APB2ENR_ADC1EN;
  GPIOA->CRL = 0xa4a24a42;
  GPIOA->CRH = 0x4444a4aa;
  GPIOB->CRL = 0x42444440;
  GPIOB->CRH = 0xaaa24a44;
  TIM1->DIER = TIM_DIER_CC4IE | TIM_DIER_UIE;
  TIM1->PSC = 3;                        //18 MHz = 72/(3+1)
  TIM1->ARR = 44999;                    //2.5 ms
  TIM1->CCMR1 = 0x6868;                 //ch1 and ch2: PWM preload
  TIM1->CCMR2 = 0x6803;                 //ch4: PWM; ch3: idle
  TIM1->BDTR = TIM_BDTR_MOE;            //master output enable
  TIM1->CCER = TIM_CCER_CC3P;           //falling edge
  TIM1->CCR4 = 5000;          //servo range: 10000 ~ 44000
  SPI1->CR2 = SPI_CR2_RXNEIE;
  SPI1->CR1 = SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_CPHA |
              SPI_CR1_BR | SPI_CR1_SSM | SPI_CR1_SSI |
              SPI_CR1_LSBFIRST;
  USART2->BRR = USART3->BRR = 312;      // 36000000/115200
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE;
  ADC1->SMPR2 = 0x07000000;             //SMP8=111 (239.5 cycles)
  ADC1->SQR3 = 8;                       //ch8
  TIM2->SMCR = TIM4->SMCR = 0x67;       //TI2FP2
  TIM2->CCMR1 = TIM2->CCMR1 = 0x100;    //CC2 as input, IC2 mapped on TI2
  NVIC_EnableIRQ(TIM1_UP_IRQn);
  NVIC_EnableIRQ(TIM1_CC_IRQn);
  NVIC_EnableIRQ(SPI1_IRQn);
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_EnableIRQ(USART3_IRQn);
  SPI1->CR1 |= SPI_CR1_SPE;
  USART2->CR1 |= USART_CR1_UE;
  USART3->CR1 |= USART_CR1_UE;
  TIM2->CR1 = TIM4->CR1 = TIM_CR1_CEN;  //turn on timers
  TIM1->CR1 = TIM_CR1_CEN;
}

void wait(unsigned i) { //Delay in milliseconds
  unsigned u = tick;
  for (loop(u); i--; u = tick) while (u == tick) loop(u);
}

unsigned getCount(void) { return TIM2->CNT + TIM4->CNT; }

unsigned wait_count(unsigned start, unsigned c) {
  while (getCount() - start < c) wait(0);
  return start;
}

unsigned wait_count(unsigned c) { return wait_count(getCount(), c); }

void setLEDs(char c) { leds = c; }

char getPath(void) {
  char c = flip(path, 0x81);
  c = flip(c, 0x42);
  c = flip(c, 0x24);
  return flip(c, 0x18);
}

unsigned getVoltage(void) {
  unsigned i = ADC1->DR;
  return (i * 8900) >> 12;
}

void toggleA0(void) {
  if (GPIOA->ODR & 1) GPIOA->BRR = 1;
  else GPIOA->BSRR = 1;
}

void pwmLeft(unsigned i) {    //input must be lest than 45000
  TIM1->CCR1 = i;
}

void pwmRight(unsigned i) {   //input must be lest than 45000
  TIM1->CCR2 = i;
}

void dirLeft(bool b) {             //true-forward
  TIM1->CCER &= ~(TIM_CCER_CC1NE | TIM_CCER_CC1E);
  if (b) TIM1->CCER |= TIM_CCER_CC1E;
  else TIM1->CCER |= TIM_CCER_CC1NE;
}

void dirRight(bool b) {            //true-forward
  TIM1->CCER &= ~(TIM_CCER_CC2NE | TIM_CCER_CC2E);
  if (b) TIM1->CCER |= TIM_CCER_CC2NE;
  else TIM1->CCER |= TIM_CCER_CC2E;
}

extern "C" void TIM1_UP_IRQHandler(void) {
  left_cnt = TIM2->CNT; right_cnt = TIM4->CNT;
  spi_cnt = 4;
  SPI1->DR = leds;
  SPI1->CR2 |= SPI_CR2_TXEIE;
  TIM1->SR &= ~TIM_SR_UIF;
  sr04(tick);
}

extern "C" void SPI1_IRQHandler(void) {
	
  char c;
  if (SPI1->CR2 & SPI_CR2_TXEIE)
    if (SPI1->SR & SPI_SR_TXE) {
      if (--spi_cnt) SPI1->DR = 0xff;
      else { SPI1->DR = 0; SPI1->CR2 &= ~SPI_CR2_TXEIE; }
    }
  if (SPI1->SR & SPI_SR_RXNE) {
    c = SPI1->DR;
    if (spi_cnt == 3) GPIOA->BRR = GPIOA->BSRR = 0x10; asm("nop");
    if (SPI1->SR & SPI_SR_TXE) {
      path = c;
      ++tick;             //whenever tick updated, path is also updated.
    }
  }
	
	

	spiValue = (char) SPI_I2S_ReceiveData(SPI1) & 0xff;
	
	
}


void Delay (int k)
{
	int x = 0;
	for (int i = 0; i <= k; i++)
		x ^= 1;
}