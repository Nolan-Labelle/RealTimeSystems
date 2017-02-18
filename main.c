#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f4xx_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f4xx_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f4xx_syscfg.h"           // Keil::Device:StdPeriph Drivers:SYSCFG

#define SELECTOR 0
#define BREWING 1
#define TRUE 1
#define FALSE 0

#define COFFEE_TIME 5
#define ESPRESSO_TIME 10
#define LATTE_TIME 15
#define MOCHA_TIME 20

static int led = 12;
static int seconds = 0;
static int tenths = 1;
static int timerRunning = FALSE;
static int mode = SELECTOR;
static int presses = 0;
static int buttonTimer = 0;


void initTimers()
{
	TIM_TimeBaseInitTypeDef timer_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	timer_InitStructure.TIM_Prescaler = 83; //Timer now scaled to 1MHz
	timer_InitStructure.TIM_Period = 100000; //Cycle how many MHz
	timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timer_InitStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //Enables interrupts?
	
	TIM_TimeBaseInit(TIM5, &timer_InitStructure);
	TIM_Cmd(TIM5, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

void initLEDs()
{
	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //initial state of button pin
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_Initstructure);
}

void enableTIM2Interrupt()
{
	NVIC_InitTypeDef nvicStructure;
	
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

void enableTIM5Interrupt()
{
	NVIC_InitTypeDef nvicStructure;
	
	nvicStructure.NVIC_IRQChannel = TIM5_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

void InitButton() // initialize user button 
{
	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);
}

// customize the EXTI for button 
void InitEXTI()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	
	// Selects the GPIOA pin 0 used as external interrupt source
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
}

void EnableEXTIInterrupt()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler () //This is the timer interrupt handler!
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		tenths++;
		
		if (tenths > 10 && seconds > 0)
		{
			seconds--;
			tenths = 1;
		}
		if (seconds == 0)
		{
			timerRunning = FALSE;
			mode = SELECTOR;
			//add code for play noise here, and we're golden
		}
		
		if (mode == BREWING && timerRunning)
		{
			if (tenths == 1)
			{
				switch (led)
				{
					case 12:
						GPIO_SetBits(GPIOD, GPIO_Pin_12);
						break;
					case 13:
						GPIO_SetBits(GPIOD, GPIO_Pin_13);
						break;
					case 14:
						GPIO_SetBits(GPIOD, GPIO_Pin_14);
						break;
					case 15:
						GPIO_SetBits(GPIOD, GPIO_Pin_15);
						break;
				}
			} 
			else 
			{
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			}
		}
		
	}
}

void TIM5_IRQHandler () //This is the timer interrupt handler!
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		if(buttonTimer > 0)
		{
			buttonTimer--;
		}
		if(buttonTimer == 0)
		{
			if(presses == 1)
			{
				if (mode == SELECTOR)
				{
					led++;
					if (led > 15)
					{
						led = 12;
					}
				}
				else if (mode == BREWING)
				{
					switch(led)
					{
						case 12:
							seconds = COFFEE_TIME;
							break;
						case 13:
							seconds = ESPRESSO_TIME;
							break;
						case 14:
							seconds = LATTE_TIME;
							break;
						case 15:
							seconds = MOCHA_TIME;
							break;
					}
				}
			}
			else if(presses > 1)
			{
				if(mode == BREWING)
				{
					mode = SELECTOR;
				}
				else if(mode == SELECTOR)
				{
					mode = BREWING;
				}
			}
			presses = 0;
		}
	}
}

void EXTI0_IRQHandler()
{
	// Checks whether the interrupt from EXTI0 or not
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		if(presses <= 0){
			buttonTimer = 10;
		}
		presses++;
	}
	
	// Clears the EXTI line pending bit
	EXTI_ClearITPendingBit(EXTI_Line0);
}

int main ()
{
	initLEDs();
	initTimers();
	enableTIM2Interrupt();
	enableTIM5Interrupt();
	
	InitButton();
	InitEXTI();
	EnableEXTIInterrupt();
	
	led = 12;
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	while (1)
	{
		if (mode == SELECTOR)
		{
			switch(led)
			{
				case 12:
					GPIO_ResetBits(GPIOD, GPIO_Pin_15);
					GPIO_SetBits(GPIOD, GPIO_Pin_12);
					break;
				case 13:
					GPIO_ResetBits(GPIOD, GPIO_Pin_12);
					GPIO_SetBits(GPIOD, GPIO_Pin_13);
					break;
				case 14:
					GPIO_ResetBits(GPIOD, GPIO_Pin_13);
					GPIO_SetBits(GPIOD, GPIO_Pin_14);
					break;
				case 15:
					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
					GPIO_SetBits(GPIOD, GPIO_Pin_15);
					break;
			}
		} 
		else if (mode == BREWING)
		{
			if (!timerRunning)
			{
				tenths = -1;
				switch(led)
				{
					case 12:
						seconds = COFFEE_TIME;
						break;
					case 13:
						seconds = ESPRESSO_TIME;
						break;
					case 14:
						seconds = LATTE_TIME;
						break;
					case 15:
						seconds = MOCHA_TIME;
						break;
				}
				timerRunning = TRUE;
			}
		}
	}
	//We never get here.
}
