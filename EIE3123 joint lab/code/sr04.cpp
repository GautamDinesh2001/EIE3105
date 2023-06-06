#include <stm32f10x.h>
#include "os.h"

#define PWM 0x6870
#define CAPTURE 0x6801
#define IDLE 0x6803

namespace {

unsigned count, distance, phase;
bool mute;

}

void pwmServo(unsigned i) {
  if (i < 10000) i = 5000;
  if (i > 44000) i = 44000;
  TIM1->CCR4 = i;
}

unsigned getDistance(void) { return distance / 105; }  // in mm

void sr04_mute(bool b) { mute = b; }

void sr04(unsigned t) {
  count++;
  if (TIM1->CCMR2 == PWM) {
    count = 0;
    TIM1->CCER &= ~TIM_CCER_CC3NE;         //disable o/p pin
    TIM1->CCMR2 = CAPTURE;
    TIM1->CCER |= TIM_CCER_CC3E;           //enable capture 
    TIM1->SR &= ~TIM_SR_CC3IF;
    if (!mute) TIM1->DIER |= TIM_DIER_CC3IE; //enable interrupt
  }
  if (!(phase = t & 7)) {
    if (TIM1->CCMR2 == CAPTURE) distance = 0;
    TIM1->DIER &= ~TIM_DIER_CC3IE;         //disable interrupt
    TIM1->CCER &= ~TIM_CCER_CC3E;          //disable capture 
    TIM1->CCR3 = 44820;                    //10us
    TIM1->CCMR2 = PWM;
    TIM1->CCER |= TIM_CCER_CC3NE;          //enable o/p pin
  }
}

extern "C" void TIM1_CC_IRQHandler(void) {
  if ((TIM1->SR & TIM_SR_CC3IF) && (TIM1->DIER & TIM_DIER_CC3IE)) {
    distance = (count * 45000) + TIM1->CCR3 - 39345;
    TIM1->DIER &= ~TIM_DIER_CC3IE;           //disable interrupt
    TIM1->CCER &= ~TIM_CCER_CC3E;            //disable capture
    TIM1->CCMR2 = IDLE;
  }
  if (TIM1->SR & TIM_SR_CC4IF) {
    if ((TIM1->CCR4 > 9999) && (!phase)) TIM1->CCER |= TIM_CCER_CC4E;
    else TIM1->CCER &= ~TIM_CCER_CC4E;
    TIM1->SR &= ~TIM_SR_CC4IF;
  }
}