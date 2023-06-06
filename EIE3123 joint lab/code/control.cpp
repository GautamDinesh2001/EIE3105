#include <stm32f10x.h>
#include "stdio.h"
#include "os.h"
#include "wheels.h"
void controller(void)
{
	char msg[50];
	float targetVelocity = 1;
	int leftCount;
	int rightCount;
	int countDif;
	int prev_countDif = 0;
	int prev_leftPWM = 12000;
	int prev_rightPWM = 12000;
	float prev_leftError = 0;
	float prev_rightError = 0;
	float prevOutput = 0;
	double k1 = 0.56;
	double k2 = 0.56;
	double k3  = 1140;
	double k4  = 1140;
	float bias = 1200;
  leftCount = getLeft();
  rightCount = getRight();
  countDif = leftCount-rightCount;
  
  float output = prevOutput + k1*countDif - k2*prev_countDif;
	
	float leftError = (targetVelocity - output) - leftCount;
	float rightError = (targetVelocity + output) - rightCount;
	
	float leftPWM = prev_leftPWM + k3*leftError - k4*prev_leftError;
	float rightPWM = prev_rightPWM +k3*rightError + k4*prev_rightError;
	
	prev_countDif = countDif;
	prev_leftError = leftError;
	prev_rightError = rightError;
	prev_leftPWM = leftPWM;
	prev_rightPWM = rightPWM;
	prevOutput = output;
	
	while(1){
	pwmLeft(leftPWM - bias);
	pwmRight(rightPWM);
	dirLeft(true);
	dirRight(true);
	sprintf(msg, "R_COUNT:%d \r\n", getRight());
	uart2_print (msg);
		         
	}
	
	
	
}