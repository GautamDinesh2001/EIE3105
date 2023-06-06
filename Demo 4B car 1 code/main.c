//Gautam Dinesh 20040968D

#include "stm32f10x.h"
#include <stdbool.h>
static unsigned char reday_to_receive=0,count=0,state=1;
static unsigned char c,character;
static unsigned char reading;
static __IO uint32_t msTicks;
__IO u32 pulseWidth = 0;
__IO u32 captureValue = 0;
__IO u32 distance = 100;
int countT=20000;
__IO bool pulseHigh = false;
volatile uint32_t wheel_count_RIGHT = 0;
volatile uint32_t wheel_count_LEFT = 0;
char buffer[50] = {'\0'};
enum direction {
    FORWARD = true,
    BACKWARD = false
};

void settingInit();
void counterInit();
void TIM1_CH3_CH3N_init();
void DelayMs(uint32_t ms);
void readFloor(void);
void LED8(unsigned char ch);
void leftWheel(int speed, bool direction);
void rightWheel(int speed, bool direction);
void lnrWheel(int speed, bool direction);
void USARTSend(char *pucBuffer, unsigned long ulCount);
void startGettingDistance();
void turn180();
TIM_OCInitTypeDef CH1;
TIM_OCInitTypeDef CH2;

int main(void)
{
	settingInit();
	counterInit();
	wheel_count_LEFT=0;
	wheel_count_RIGHT=0;
  while(1)
  {
		readFloor();
		LED8(reading);
		if((reading&(1<<3)&&reading&(1<<4)&&reading&(1<<5)&&reading&(1<<6)&&reading&(1<<7))||(reading&(1<<0)&&reading&(1<<1)&&reading&(1<<2)&&reading&(1<<3)&&reading&(1<<4))){
			if(countT>3000){
				countT=0;
				count++;
			}
			if(count==1){
				leftWheel(3000,FORWARD);
				rightWheel(3000,FORWARD);
			}else if(count==3){
				leftWheel(3500,FORWARD);
				rightWheel(3500,BACKWARD);
				DelayMs(450);
				lnrWheel(0,FORWARD);
				state=0;
				count++;
			}
			if(count==4){
				DelayMs(9000);
				startGettingDistance();
				while(count==4){
					startGettingDistance();
					if(distance<35){
						count++;
						leftWheel(5600,BACKWARD);
						rightWheel(5600,FORWARD);
						DelayMs(800);
						while(!reading&(1<<7)){
							readFloor();
							LED8(reading);
							leftWheel(3000,BACKWARD);
							rightWheel(3000,FORWARD);
						}
						state=2;
					}
				}
			}else{
				lnrWheel(3100,FORWARD);
				DelayMs(100);
				state=1;
			}
		}else if(state==1&&count!=3){
			lnrWheel(3100,FORWARD);
			if(reading&(1<<4)||reading&(1<<5)||reading&(1<<6)){	//e.g.00001111
				state=3;
			}
			if(reading&(1<<3)||reading&(1<<2)||reading&(1<<1)){	//e.g.11110000
				state=2;
			}
		}else if(state==2){	//turnLeft
			rightWheel(4100,FORWARD);
			leftWheel(3100,FORWARD);
			if(reading&(1<<2)){
				rightWheel(4400,FORWARD);//
				leftWheel(2300,FORWARD);//
			}
			if(reading&(1<<1)){
				rightWheel(3900,FORWARD);//
				leftWheel(1700,BACKWARD);//
			}
			if(reading==0){
				rightWheel(3700,FORWARD);//
				leftWheel(1500,BACKWARD);//
			}
			 DelayMs(50);
			lnrWheel(3300,FORWARD);
			if((reading&(1<<2)&&!reading&(1<<1))||reading&(1<<3)||reading&(1<<4)){
				state=1;
			}
		}else if(state==3){	//turnRight
			rightWheel(2100,FORWARD);
			leftWheel(4700,FORWARD);
			if(reading&(1<<5)){
				rightWheel(2100,FORWARD);
				leftWheel(3600,FORWARD);
			}
			if(reading&(1<<6)){
				rightWheel(1700,BACKWARD);
				leftWheel(3300,FORWARD);
			}
			if(reading==0){
				rightWheel(1100,BACKWARD);
				leftWheel(2600,FORWARD);
			}
			 DelayMs(50);
			lnrWheel(3100,FORWARD);
			if(reading&(1<<3)||reading&(1<<4)||(reading&(1<<5)&&!reading&(1<<6))){
				state=1;
			}
		}
	}
}

void turn180(){
	wheel_count_LEFT=0;
	wheel_count_RIGHT=0;
	while(wheel_count_LEFT<50&&wheel_count_RIGHT<50){
		if(wheel_count_LEFT<50){
			leftWheel(4000,FORWARD);
		}else{
			leftWheel(0,BACKWARD);
		}	
		if(wheel_count_RIGHT<50){
			rightWheel(4000,BACKWARD);
		}else{
			rightWheel(0,FORWARD);
		}
	}
}

void startGettingDistance(){
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		DelayMs(10);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
}

void USARTSend(char *pucBuffer, unsigned long ulCount)
{
	//loop until the whole string is sent
	while(ulCount--)
	{
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		{
    }
		USART_SendData(USART3, *pucBuffer++);
		/* loop until the end of transmission */			
  }
}

// TIM1 Channel 3 and Channel 3N initialization
void TIM1_CH3_CH3N_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// config TIM1 CH3 I/O
	// echo inpute capture
	GPIO_InitTypeDef init_gpio;
	init_gpio.GPIO_Mode = GPIO_Mode_IPD; // the mode is not AF_PP
	init_gpio.GPIO_Pin = GPIO_Pin_10; // TIM1 CH3 pin
	init_gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &init_gpio);

	//set TIM1 in 1000Hz
	//pulse range: 0 to 1000
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// 1ms per cycle, 1000Hz
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	timerInitStructure.TIM_Prescaler = 72-1; // 1(72MHz/72) = 1us, 72
	timerInitStructure.TIM_Period = 10000-1; // 1us*10000 = 10ms, 10000
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &timerInitStructure);
	TIM_Cmd(TIM1, ENABLE);
 
	//enable TIM1 CH3 Input Capture
	TIM_ICInitTypeDef inputCapture_init;
	inputCapture_init.TIM_Channel = TIM_Channel_3; //seclect IC1
	inputCapture_init.TIM_ICPolarity = TIM_ICPolarity_Rising; //capture rising edge
	inputCapture_init.TIM_ICSelection = TIM_ICSelection_DirectTI; //map to TI3
	inputCapture_init.TIM_ICPrescaler = TIM_ICPSC_DIV1; //configure input frequency
	inputCapture_init.TIM_ICFilter = 0; //no filter
	TIM_ICInit(TIM1, &inputCapture_init);
  
	//enable input capture interrput
	NVIC_InitTypeDef nvic_init;
	nvic_init.NVIC_IRQChannel = TIM1_CC_IRQn; //TIM1 capture compare interrput
	//preemptive priority level 2
	nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
	//from the priority level 0
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	//the IRQ channel is enabled
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
	//allow update to interrput, allows the CC3IE to capture interrput
	TIM_ITConfig(TIM1, TIM_IT_Update|TIM_IT_CC3, ENABLE);
}

void TIM1_CC_IRQHandler(void)
{
    	if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)
	{
      		if(!pulseHigh)
		{
        		pulseHigh = true;   //pulse starts
        		TIM_SetCounter(TIM1, 0);
		        //change to detect the falling edge
        		TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling);
	      	}
		else
		{
        		pulseWidth += TIM_GetCounter(TIM1);
        		captureValue = pulseWidth;
			// output an integer only not a floating point number
			// very similar to captureValue/2*330/1000000
        		distance = (captureValue/2)/30; // in cm
        
        		//change to detect the rising edge
	        	TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising);
		        pulseHigh = false;
		        pulseWidth = 0;
	      	}
    	}
	//clear interrupt flag
  	TIM_ClearITPendingBit(TIM1, TIM_IT_Update|TIM_IT_CC3);
}



void counterInit(){
 		//WHEEL_COUNT
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);
		// Configure TIM2 and TIM4 timers
		TIM_TimeBaseInitTypeDef WHEEL_COUNT;
    WHEEL_COUNT.TIM_Period = 65535;
    WHEEL_COUNT.TIM_Prescaler = 71;
    WHEEL_COUNT.TIM_ClockDivision = 0;
    WHEEL_COUNT.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &WHEEL_COUNT);
    TIM_TimeBaseInit(TIM4, &WHEEL_COUNT);
		
		// Configure TIM2 channel 2 and TIM4 channel 2 as input capture channels
		TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
		
		// Enable interrupts for TIM2 and TIM4 input capture channels
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
		
		// Configure the NVIC for TIM2 and TIM4 interrupts
		NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // Start the TIM2 and TIM4 timers
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
		
}



void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
        wheel_count_LEFT++;
    }
}

void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
        wheel_count_RIGHT++;
    }
}


void leftWheel(int speed, bool direction) {
    // Set the PWM mode and pulse width for the left wheel
    CH1.TIM_OCMode = TIM_OCMode_PWM1;
    CH1.TIM_Pulse = speed;
    if (direction) {
			// Move the wheel forward
			CH1.TIM_OutputState = TIM_OutputState_Enable;
			CH1.TIM_OCPolarity = TIM_OCPolarity_High;
			CH1.TIM_OutputNState = TIM_OutputNState_Disable;
			CH1.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    } else {
			// Move the wheel backward
			CH1.TIM_OutputState = TIM_OutputState_Disable;
			CH1.TIM_OCPolarity = TIM_OCPolarity_Low;
			CH1.TIM_OutputNState = TIM_OutputNState_Enable;
			CH1.TIM_OCNPolarity = TIM_OCNPolarity_High;
    }

    // Initialize the PWM for the left wheel
    TIM_OC1Init(TIM1, &CH1); //PA8 
}

void rightWheel(int speed, bool direction) {
    // Set the PWM mode and pulse width for the right wheel
    CH2.TIM_OCMode = TIM_OCMode_PWM1;
    CH2.TIM_Pulse = speed;

    if (direction) {
			// Move the wheel forward
			CH2.TIM_OutputState = TIM_OutputState_Disable;
			CH2.TIM_OCPolarity = TIM_OCPolarity_Low;
			CH2.TIM_OutputNState = TIM_OutputNState_Enable;
			CH2.TIM_OCNPolarity = TIM_OCNPolarity_High;
    } else {
			// Move the wheel backward
			CH2.TIM_OutputState = TIM_OutputState_Enable;
			CH2.TIM_OCPolarity = TIM_OCPolarity_High;
			CH2.TIM_OutputNState = TIM_OutputNState_Disable;
			CH2.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    }

    // Initialize the PWM for the right wheel
    TIM_OC2Init(TIM1, &CH2); //PA9
}

void lnrWheel(int speed, bool dir){
	leftWheel(speed,dir);
	rightWheel(speed,dir);
}

void settingInit(){
	
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	
	// Configure the SysTick timer to overflow every 1 ms
	SysTick_Config(SystemCoreClock / 1000);
	
  // Enable USART2 and GPIOB clocks
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
  // Configure PA2 and PA3 as USART2 pins
  GPIO_InitTypeDef GPIO_USART2;
  GPIO_USART2.GPIO_Pin = GPIO_Pin_2;
  GPIO_USART2.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_USART2.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_USART2);

  GPIO_USART2.GPIO_Pin = GPIO_Pin_3;
  GPIO_USART2.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_USART2);
	
  USART_InitTypeDef USART_Init2;
  USART_Init2.USART_BaudRate = 115200;
  USART_Init2.USART_WordLength = USART_WordLength_8b;
  USART_Init2.USART_StopBits = USART_StopBits_1;
  USART_Init2.USART_Parity = USART_Parity_No;
  USART_Init2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init2.USART_Mode = USART_Mode_Tx;
  USART_Init(USART2, &USART_Init2);

  // Enable USART2
  USART_Cmd(USART2, ENABLE);
	
			//mycode
	//set up for USART3

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	// Tx3(PB10)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//RX3(PB11)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	//USART3 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	//USART_ClockInitTypeDef USART_ClockInitStructure; 
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	
  //pwm
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  TIM_TimeBaseInitTypeDef timerInitStructure; 
  timerInitStructure.TIM_Prescaler = 72-1;//1/(72Mhz/720)=0.01ms=0.00001s
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 10000-1;//0.00001s*200=0.002s=1/500Hz
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &timerInitStructure);
  TIM_Cmd(TIM1, ENABLE);
  TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
  TIM_CtrlPWMOutputs(TIM1,ENABLE);
	CH1.TIM_OCMode = TIM_OCMode_PWM1;
	
	CH1.TIM_Pulse =1-1;// left wheel
	CH1.TIM_OutputState = TIM_OutputState_Enable;	//moveforward
	CH1.TIM_OCPolarity = TIM_OCPolarity_High;
	//CH1.TIM_OCIdleState=TIM_OCIdleState_Reset;
	CH1.TIM_OutputNState = TIM_OutputNState_Disable;//Nstate is for backward
	CH1.TIM_OCNPolarity = TIM_OCNPolarity_Low;//N
	//CH1.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//N
	TIM_OC1Init(TIM1, &CH1);//PA8 
	CH2.TIM_OCMode = TIM_OCMode_PWM1;
	CH2.TIM_Pulse =1-1;// right wheel
	CH2.TIM_OutputState = TIM_OutputState_Disable;
	CH2.TIM_OCPolarity = TIM_OCPolarity_Low;
	//CH2.TIM_OCIdleState=TIM_OCIdleState_Reset;
	CH2.TIM_OutputNState = TIM_OutputNState_Enable;//N
	CH2.TIM_OCNPolarity = TIM_OCNPolarity_High;//N
	//CH2.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//N
	TIM_OC2Init(TIM1, &CH2);//PA9
	
	//ultrasonic sensor
	TIM1_CH3_CH3N_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	//pulse_generate(10);
	// config TIM1 CH3N I/O (PB15) BUT do not enable TIM1 CH3N
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef init_gpio;
	init_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	init_gpio.GPIO_Pin = GPIO_Pin_15;
	init_gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &init_gpio);
}

void USART3_IRQHandler() {
	
	if(USART_GetITStatus(USART3, USART_IT_TC) != RESET) 
	{	
		USART_ClearITPendingBit(USART3, USART_IT_TC);
	}
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		character = (unsigned char) USART_ReceiveData(USART3); 
		USART_SendData(USART3, character);
	}
}


void LED8(unsigned char ch)
{
	
	GPIO_AFIODeInit();//Deinitializes the Alternate Functions
	
	SPI_InitTypeDef   SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// Enable SPI1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1|RCC_APB2Periph_AFIO, ENABLE);
	
	//SPIx_SCK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//SPIx_MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//SPIx_MISO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//SPIx_NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	SPI_I2S_DeInit(SPI1);
	// SPI initialization
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

	SPI_Cmd(SPI1, ENABLE);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);	
	SPI_I2S_SendData(SPI1,ch);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);	
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
	SPI_Cmd(SPI1, DISABLE);
	
}	

void readFloor() 
{
	reday_to_receive=1;
	// Setup PA5 and PA7
	// PA5 = SPI1_SCK, PA7 = IR LED / MODE
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_SPI1EN, ENABLE);

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	// 36 MHz / 256 = 140.625 kHz
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_Init(SPI1, &SPI_InitStructure);
	// Enable the receive interrupt
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);

	// Set PA7 to 1 around 3ms
	for(long int o = 0;o < 216000; o++)//72Mhz/1000*3
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
	}
	// Initialize the data transmission from the master to the slave
	SPI_I2S_SendData(SPI1, 0);
	// Enable the interrupt to receive data by using the ISR handler
	NVIC_EnableIRQ(SPI1_IRQn);
	while (SPI1->SR & SPI_SR_BSY);
}

// put the readings to the variable c
void SPI1_IRQHandler() 
{
	while (SPI1->SR & SPI_SR_BSY);
		// the received character has all the readings
		 c = (char) SPI_I2S_ReceiveData(SPI1) & 0xff;
		// Check PA7. If it is 1, it means the data is ready
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) == 1) 
	{
		// Set PA7 to 0 to trigger the shift register
		GPIO_ResetBits(GPIOA, GPIO_Pin_7);
		// Go to get the next reading
		SPI_I2S_SendData(SPI1, 0);
	} 
	else 
	{
		//USART2
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_SendData(USART2, c);
	
		reading = c;
		reday_to_receive = 0;
		// disable the interrupt because it is not ready
		NVIC_DisableIRQ(SPI1_IRQn);
		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);
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
	countT++;
}
