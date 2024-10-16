#include "stm32f10x.h"
unsigned char state=0;
unsigned char c;
static __IO uint32_t msTicks;
void DelayMs(uint32_t ms);
void readFloor();
unsigned char led8bit=1;
void SPI_init();
void bluetooth();
int line;
int lastLine = 0;
int once = 0, count1 = 0, count2 = 0, count3 = 0;
int setspeed = 18;
TIM_OCInitTypeDef outputChannelInit;

void pwm_init(){
	//set up for Periph Bus
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// SPI1 initialization
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_SPI1EN, ENABLE);
	
	SPI_InitTypeDef   SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;// full duplex in 2 lines
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//master can control data transfer
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	//8bit datasize
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	// communication is high when ready
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//data is capture on second edge
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	//NSS signal is managed by software(use program code to send and receive data)
	// 36 MHz / 256 = 140.625 kHz
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//data transfer start from MSB
	SPI_Init(SPI1, &SPI_InitStructure);

	// Enable the receive interrupt
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);
	
	
	//GPIO set up for PA8(green/Left wheel Forward) PA9(blue/Right wheel Backward) PA10(red/bootloader dont touch) 
	//GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO set up for PB13/PB14 (left/right wheel backward)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PB6 LED
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	//Timer 1 set up (PWM, left and right wheels) 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 720-1;//1/(72Mhz/720)=0.01ms=0.00001s
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 200-1;//0.00001s*200=0.002s=1/500Hz
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &timerInitStructure);
	TIM_Cmd(TIM1, ENABLE);
	//set up for PWM CH1
	TIM_OCInitTypeDef outputChannelInit;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;	//active when TIM1_CounterRegister<TIM1_CaptureCompareRegister
	outputChannelInit.TIM_Pulse = 1-1; 	//pulse value loaded into Capture Compare Register
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;	//ENABLE timer outpur
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;	//SET high to active
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1
	
	
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
		
  TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

void forward(int speed){
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;	//active when TIM1_CounterRegister<TIM1_CaptureCompareRegister
	outputChannelInit.TIM_Pulse = speed; 	//pulse value loaded into Capture Compare Register
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;	//SET high to active
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1
	
	outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}
void backward(int speed){
	outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;	//active when TIM1_CounterRegister<TIM1_CaptureCompareRegister
	outputChannelInit.TIM_Pulse = speed+4; 	//pulse value loaded into Capture Compare Register
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;	//SET high to active
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1

	outputChannelInit.TIM_Pulse = speed+18;
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}
void left(int speed, int turn){
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;	//active when TIM1_CounterRegister<TIM1_CaptureCompareRegister
	outputChannelInit.TIM_Pulse = speed+setspeed; 	//pulse value loaded into Capture Compare Register
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;	//SET high to active
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1
	
	outputChannelInit.TIM_Pulse = speed + setspeed + turn;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}
void left_rotate(int speed){
	outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;	//active when TIM1_CounterRegister<TIM1_CaptureCompareRegister
	outputChannelInit.TIM_Pulse = speed + 10; 	//pulse value loaded into Capture Compare Register
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;	//SET high to active
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1
	
	outputChannelInit.TIM_Pulse = speed;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}
void right_rotate(int speed){
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;	//active when TIM1_CounterRegister<TIM1_CaptureCompareRegister
	outputChannelInit.TIM_Pulse = speed; 	//pulse value loaded into Capture Compare Register
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;	//SET high to active
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1
	
	outputChannelInit.TIM_Pulse = speed;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;	//ENABLE timer outpur
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}
void right(int speed, int turn){
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;	//ENABLE timer outpur
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;	//active when TIM1_CounterRegister<TIM1_CaptureCompareRegister
	outputChannelInit.TIM_Pulse = speed + setspeed + turn; 	//pulse value loaded into Capture Compare Register
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;	//SET high to active
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1

	outputChannelInit.TIM_Pulse = speed + setspeed;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;	//ENABLE timer outpur
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}
void stop(){
	outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;
	outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
	
	TIM_OC1Init(TIM1, &outputChannelInit);	//put it to TIM1_CH1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);	//Enable peripheral perload register on CCR1
	TIM_OC2Init(TIM1,&outputChannelInit);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
}

int main(void)
{
	pwm_init();
	SPI_init();
	//bluetooth();
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	
	// Configure the SysTick timer to overflow every 1 ms
	SysTick_Config(SystemCoreClock / 1000);

	while(1)
	{
		readFloor();
		DelayMs(3);//can not too fast, maybe need to clear some flag
		
		//Change state when checkpoint reached
		if(line == 0x00){
			if(once == 0){
				state++;
				once = 1;
			}
		}else{once = 0;}
		
		//A -> B
		if(state==0){
			//Track line
			if((line&(1<<7))==0){
				left(20,55);
			}else if((line&(1<<6))==0){
				left(35,25);
			}else if((line&(1<<5))==0){
				left(45,15);
			}else if((line&0b11000)==0){
				left(50,8);
			}
			//Turn right
			if((line&(1<<0))==0){
				right(30,10);
			}else if((line&(1<<1))==0){
				right(35,8);
			}else if((line&(1<<2))==0){
				right(35,4);
			}else if((line&0b11000)==0){
				left(50,5);
			}
			count2 = 0;
		}
		//B -> C
		if(state==1){
			if(count2 < 180){
				if((line&(1<<7))==0){
					left(30,50);
				}else if((line&(1<<6))==0){
					left(30,35);
				}else if((line&(1<<5))==0){
					left(40,15);
				}else if((line&0b11000)==0){
					left(45,10);
				}
				count2++;
			}else{
				if((line&(1<<7))==0){
					left(13,70);
				}else if((line&(1<<6))==0){
					left(20,45);
				}else if((line&0b11000)==0){
					left(25,35);
				}
			}
			//Turn right
			if((line&(1<<0))==0){
				right(30,20);
			}else if((line&(1<<1))==0){
				right(30,10);
			}else if((line&(1<<2))==0){
				right(35,5);
			}
		}
		//-----Half round-----
		//C -> D
		if(state==2){
			//Track line
			if(once==0){
				if((line&(1<<7))==0){
					left(20,55);
				}else if((line&(1<<6))==0){
					left(35,25);
				}else if((line&(1<<5))==0){
					left(45,15);
				}
			}
			if((line&0b11000)==0){
				left(40,8);
			}
			//Turn right
			if((line&(1<<0))==0){
				right(30,10);
			}else if((line&(1<<1))==0){
				right(40,8);
			}else if((line&(1<<2))==0){
				right(40,4);
			}
			count2 = 0;
		}
		//D -> A
		if(state==3){
			if(count2 < 170){
				if((line&(1<<7))==0){
					left(35,55);
				}else if((line&(1<<6))==0){
					left(35,40);
				}else if((line&(1<<5))==0){
					left(50,10);
				}else if((line&0b11000)==0){
					left(53,8);
				}
				count2++;
			}else{
				if((line&(1<<7))==0){
					left(18,45);
				}else if((line&0b11000)==0){
					left(40,15);
				}
			}
			//Turn right
			if((line&(1<<0))==0){
				right(20,25);
			}else if((line&(1<<1))==0){
				right(20,15);
			}else if((line&(1<<2))==0){
				right(25,10);
			}
		//<No right>
		}
		//==========1st loop==========
		//A -> X -> C
		if(state==4){
			//Track line
			if(line < 0b11110000){
				if((line&(1<<7))==0){
					left(18,65);
				}else if((line&(1<<6))==0){
					left(15,56);
				}else if((line&(1<<5))==0){
					left(20,48);
				}
			}else{
				//Turn right
				if((line&(1<<0))==0){
					right(5,65);
				}else if((line&(1<<1))==0){
					right(15,46);
				}else if((line&(1<<2))==0){
					right(20,38);
				}
			}
			count2 = 0;
		}
		//C -> D
		if((state >= 5) && (state <= 7)){
			//Prevent random state at C
			if(count2 < 180) count2++;
			else state = 7;
			
			//Track line
			if((line&(1<<7))==0){
				left(15,50);
			}else if((line&(1<<6))==0){
				left(25,30);
			}else if((line&(1<<5))==0){
				left(50,15);
			}else if((line&0b11000)==0){
				left(55,8);
			}
			//Turn right
			if((line&(1<<0))==0){
				right(30,10);
			}else if((line&(1<<1))==0){
				right(35,8);
			}else if((line&(1<<2))==0){
				right(35,4);
			}
		}
		//D -> A
		if(state==8){
			if(count2 < 180){
				if((line&(1<<7))==0){
					left(35,50);
				}else if((line&(1<<6))==0){
					left(35,35);
				}else if((line&(1<<5))==0){
					left(40,15);
				}else if((line&0b11000)==0){
					left(50,10);
				}
				count2++;
			}else{
				if((line&(1<<7))==0){
					left(18,50);
				}else if((line&(1<<6))==0){
					left(25,40);
				}else if((line&0b11000)==0){
					left(25,30);
				}
			}
			//Turn right
			if((line&(1<<0))==0){
				right(30,20);
			}else if((line&(1<<1))==0){
				right(30,10);
			}else if((line&(1<<2))==0){
				right(35,5);
			}
		}
		//-----Half round-----
		//A -> B
		if(state==9){
			//Track line
			if((line&(1<<7))==0){
				left(20,55);
			}else if((line&(1<<6))==0){
				left(35,25);
			}else if((line&(1<<5))==0){
				left(45,15);
			}else if((line&0b11000)==0){
				left(50,8);
			}
			//Turn right
			if((line&(1<<0))==0){
				right(30,10);
			}else if((line&(1<<1))==0){
				right(35,8);
			}else if((line&(1<<2))==0){
				right(35,4);
			}else if((line&0b11000)==0){
				left(50,5);
			}
			count2 = 0;
		}
		//B -> C
		if(state==10){
			if(count2 < 160){
				if((line&(1<<7))==0){
					left(35,50);
				}else if((line&(1<<6))==0){
					left(35,35);
				}else if((line&(1<<5))==0){
					left(40,15);
				}else if((line&0b11000)==0){
					left(50,10);
				}
				count2++;
			}else{
				if((line&(1<<7))==0){
					left(18,50);
				}else if((line&(1<<6))==0){
					left(22,40);
				}else if((line&0b11000)==0){
					left(23,30);
				}
			}
			//Turn right
			if((line&(1<<0))==0){
				right(30,20);
			}else if((line&(1<<1))==0){
				right(30,10);
			}else if((line&(1<<2))==0){
				right(35,5);
			}
		}
		//==========2nd loop==========
		//C turning
		if((state >= 11) && (state <= 14)){
			if(count3 < 165){
				left_rotate(70);
				count3++;
			}else{
				state = 15;
				count2 = 0;
				count3 = 0;
			}
		}
		if(state==15){
			if(count3 < 220){
				right(10,55);
				count3++;
			}else{
				state = 16;
				count3 = 0;
			}
		}
		//C -> X -> A
		if(state>=16 && state <=18){
			if(count2 < 300) count2++;
			else state = 16;
			//Track line
			if(line < 0b11110000){
				if((line&(1<<7))==0){
					left(0,65);
				}else if((line&(1<<6))==0){
					left(15,46);
				}else if((line&(1<<5))==0){
					left(20,38);
				}
			}else{
				//Turn right
				if((line&(1<<0))==0){
					right(5,65);
				}else if((line&(1<<1))==0){
					right(15,46);
				}else if((line&(1<<2))==0){
					right(20,38);
				}
			}
		}
		//A
		if(state==17) stop();
		//==========End==========
		//---------------STOP---------------
		if(line == 0xFF){
			if(count1 > 280){
				stop();
			}else{
				count1++;
				if((lastLine&(1<<7)) == 0) left(18,45);
				if((lastLine&(1<<0)) == 0) right(18,45);
			}
		}else{ count1 = 0; }
		
		lastLine = line;
	}
}
void readFloor() 
{
	// Set PA7 to 1
	GPIO_SetBits(GPIOA, GPIO_Pin_7);
	// New code: it can turn on Infra-red LED in 3 ms so that
	// the photoresistors can get the reading properly
	DelayMs(3);
	// Initialize the data transmission from the master to the slave
	SPI_I2S_SendData(SPI1, 0);
	// Enable the interrupt to receive data by using the ISR handler
	NVIC_EnableIRQ(SPI1_IRQn);
}
// put the readings to the variable c
void SPI1_IRQHandler() 
{

	// the received character has all the readings
	 c = (char) SPI_I2S_ReceiveData(SPI1) & 0xff;
	
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) == 1) 
	{
	  // Set PA7 to 0 to trigger the shift register
	  GPIO_ResetBits(GPIOA, GPIO_Pin_7);
	  // Go to get the next reading
	  SPI_I2S_SendData(SPI1, 0);
		
	} 
	else 
	{
	//debug and test code
		line = 0b00000000;
		{
			if(c&(1<<0))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<0);
			}
			if(c&(1<<1))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<1);
			}	
			if(c&(1<<2))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<2);
			}	
			if(c&(1<<3))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<3);
			}	
			if(c&(1<<4))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<4);
			}	
			if(c&(1<<5))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<5);
			}	
			if(c&(1<<6))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<6);
			}	
			if(c&(1<<7))
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'0');
			}
			else
			{
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
				USART_SendData(USART3,'1');
				line |= (1<<7);
			}	
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	USART_SendData(USART3, '\r');
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	USART_SendData(USART3, '\n');
	}
		
	
	// Check PA7. If it is 1, it means the data is ready
	
	  // disable the interrupt because it is not ready
	  NVIC_DisableIRQ(SPI1_IRQn);
	
	}

}

void DelayMs(uint32_t ms)
{
	// Reload ms value
	msTicks = ms;
	// Wait until msTick reach zero
	while (msTicks);
}

// SysTick_Handler function will be called every 1 ms
void SysTick_Handler()
{
	if (msTicks != 0)
	{
		msTicks--;
	}
}

void SPI_init(){
	// Setup PA5 and PA7
	// PA5 = SPI1_SCK, PA7 = IR LED / MODE
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_SPI1EN, ENABLE);

	SPI_InitTypeDef   SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  // 36 MHz / 256 = 140.625 kHz
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStructure);
	// Enable the receive interrupt
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
  // Enable SPI1
  SPI_Cmd(SPI1, ENABLE);
	
	// Enable USART3 and GPIOB clocks
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	// Configure PB10 and PB11 as USART3 pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Initialize USART3
	USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
	
	// Enable USART2 and GPIOB clocks
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// Configure PA2 and PA3 as USART2 pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  // Enable USART2,USART3
  USART_Cmd(USART3, ENABLE);
	USART_Cmd(USART2, ENABLE);
}
