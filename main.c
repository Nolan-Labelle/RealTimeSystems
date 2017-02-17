#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f4xx_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f4xx_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f4xx_syscfg.h"           // Keil::Device:StdPeriph Drivers:SYSCFG




static int led = 12;
static int seconds = 0;
static int tenths = 1;
static int brewing = 0;//0 = not brewing, 1 = brewing.

static int mode = 1; //0 = selector mode, 1 = brewing mode.

void initTimers()
{
	TIM_TimeBaseInitTypeDef timer_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	timer_InitStructure.TIM_Prescaler = 83; //Timer now scaled to 1MHz
	timer_InitStructure.TIM_Period = 1000000; //Cycle how many MHz
	timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timer_InitStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //Enables interrupts?
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

void enableTimerInterrupt()
{
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

void TIM2_IRQHandler () //This is the timer interrupt handler!
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		tenths++;
		if(tenths > 10 && seconds > 0){
			seconds--;
			tenths = 1;
		}
		if(mode == 1 && brewing){
			if(tenths == 3){
				if(led == 12){
					GPIO_SetBits(GPIOD, GPIO_Pin_12);
				} else if(led == 13){
					GPIO_SetBits(GPIOD, GPIO_Pin_13);
				} else if(led == 14){
					GPIO_SetBits(GPIOD, GPIO_Pin_14);
				} else if(led == 15){
					GPIO_SetBits(GPIOD, GPIO_Pin_15);
				}
			} else {
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			}
		}
		
	}
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

void EXTI0_IRQHandler()
{
	// Checks whether the interrupt from EXTI0 or not
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		if(led >= 15){
			led = 12;
		} else {
			led ++;
		}
		if(mode == 1){
			brewing = 0;
		}
		// Clears the EXTI line pending bit
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

int main ()
{
	initLEDs();
	initTimers();
	enableTimerInterrupt();
	
	InitButton();
	InitEXTI();
	EnableEXTIInterrupt();
	
	led = 12;
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	while (1)
	{
		if(mode == 0){
			if(led == 12){
				GPIO_ResetBits(GPIOD, GPIO_Pin_15);
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
			} else if(led == 13){
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				GPIO_SetBits(GPIOD, GPIO_Pin_13);
			} else if(led == 14){
				GPIO_ResetBits(GPIOD, GPIO_Pin_13);
				GPIO_SetBits(GPIOD, GPIO_Pin_14);
			} else if(led == 15){
				GPIO_ResetBits(GPIOD, GPIO_Pin_14);
				GPIO_SetBits(GPIOD, GPIO_Pin_15);
			}
		} else if (mode == 1){
			if(!brewing){
				tenths = 1;
				if(led == 12){
					seconds = 5;
				} else if(led == 13){
					seconds = 10;
				} else if(led == 14){
					seconds = 15;
				} else if(led == 15){
					seconds = 20;
				}
				brewing = 1;
			}
		}
		//__asm("nop"); //Just spin.
	}
}
