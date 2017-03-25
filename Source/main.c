#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "croutine.h"
#include <stdlib.h>
#include "stm32f4xx_exti.h"

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#define LED_OFF 0x00000000
#define LED_ON  0x00000001

typedef struct
{
	QueueHandle_t blue;
	QueueHandle_t green;
	QueueHandle_t orange;
	QueueHandle_t red;
} SelectorPack;

//Globals:
static int clicks = 0;
static int whichLED = LED_BLUE;
static TimerHandle_t clickTimer;
static int lastPress = 0;
static int debounceDelay = 125 / portTICK_RATE_MS;
static TickType_t thisPress;
static QueueHandle_t xBlueQueue;
static QueueHandle_t xGreenQueue;
static QueueHandle_t xOrangeQueue;
static QueueHandle_t xRedQueue;
static int* blueCmd = 0;
static int* greenCmd = 0;
static int* orangeCmd = 0;
static int* redCmd = 0;
static int* offCmd;
static int* onCmd;
static int brewTimes[4];
static int* brewToggle;

void panic() //If something has gone horribly wrong, then disco mode.
{
	STM_EVAL_LEDOn(LED_BLUE);
	STM_EVAL_LEDOn(LED_GREEN);
	STM_EVAL_LEDOn(LED_ORANGE);
	STM_EVAL_LEDOn(LED_RED);
}

void singleClick()
{	
	switch (whichLED)
	{
		case LED_BLUE:
			whichLED = 	LED_GREEN;
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOn(LED_GREEN);
			break;
		case LED_GREEN:
			whichLED = 	LED_ORANGE;
			STM_EVAL_LEDOff(LED_GREEN);
			STM_EVAL_LEDOn(LED_ORANGE);
			break;
		case LED_ORANGE:
			whichLED = 	LED_RED;
			STM_EVAL_LEDOff(LED_ORANGE);
			STM_EVAL_LEDOn(LED_RED);
			break;
		case LED_RED:
			whichLED = 	LED_BLUE;
			STM_EVAL_LEDOff(LED_RED);
			STM_EVAL_LEDOn(LED_BLUE);
			break;
		default:
			panic();
			break;
	}
	clicks = 0;
}

void doubleClick()
{
	int fail = 0;
	
	switch (whichLED)
	{
		case LED_BLUE:
			if(!xQueueSend(xBlueQueue, brewToggle, 0))
				fail++;
			break;
		case LED_GREEN:
			if(!xQueueSend(xGreenQueue, brewToggle, 0))
				fail++;
			break;
		case LED_ORANGE:
			if(!xQueueSend(xOrangeQueue, brewToggle, 0))
				fail++;
			break;
		case LED_RED:
			if(!xQueueSend(xRedQueue, brewToggle, 0))
				fail++;
			break;
		default:
			panic();
			break;
	}
	if (fail)
	{
		panic();
	}
	clicks = 0;
}

void vClickTimerHandler(TimerHandle_t xTimer)
{
	if (clicks == 1)
	{
		singleClick();
	}
	else if (clicks >= 2)
	{
		doubleClick();
	}
	else
	{
		panic();
	}
}

void EXTI0_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	thisPress = xTaskGetTickCountFromISR();
	if((thisPress - lastPress) > debounceDelay)
	{
		if (clicks == 0)
		{
			clicks++;
			if(xTimerStartFromISR(clickTimer, &xHigherPriorityTaskWoken) != pdPASS)
			{
				//Timer was not able to start
				panic();
			}
		}
		else
		{
			clicks++;
		}
	}
	lastPress = thisPress;
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void update (QueueHandle_t colorQueue, int* colorCmd, int color)
{
	if(xQueueReceive(colorQueue, colorCmd, 0))
	{
		if (*colorCmd == 1)
		{
			if(brewTimes[color])
			{
				brewTimes[color] = 0;
			}
			else
			{
				brewTimes[color] = (color+1) * 10; //G:5 B:10 R:15 O:20
			}
		}
	}
}

void vCoffeeThread (void* pvParameters)
{
	int currentBrew = 0;
	TickType_t lastWake = xTaskGetTickCount();
	brewTimes[0] = 0; //Green
	brewTimes[1] = 0; //Blue
	brewTimes[2] = 0; //Red
	brewTimes[3] = 0; //Orange
		
	for(;;)
	{
		//check each queue for updates
		update(xBlueQueue, blueCmd, LED_BLUE);
		update(xGreenQueue, greenCmd, LED_GREEN);
		update(xOrangeQueue, orangeCmd, LED_ORANGE);
		update(xRedQueue, redCmd, LED_RED);
		
		if (!(brewTimes[0] <= 0 && brewTimes[1] <= 0 && brewTimes[2] <= 0 && brewTimes[3] <= 0)) //All zeros
		{
			//find next positive coffee, will spin for entire time slice if none.
			while(brewTimes[currentBrew] <= 0)
			{
				currentBrew = (currentBrew + 1) % 4;
			}
			
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOff(LED_GREEN);
			STM_EVAL_LEDOff(LED_ORANGE);
			STM_EVAL_LEDOff(LED_RED);
			STM_EVAL_LEDOn(whichLED);
			
			switch(currentBrew)
			{
				case(0):
					STM_EVAL_LEDOn(LED_GREEN);
					break;
				case(1):
					STM_EVAL_LEDOn(LED_BLUE);
					break;
				case(2):
					STM_EVAL_LEDOn(LED_RED);
					break;
				case(3):
					STM_EVAL_LEDOn(LED_ORANGE);
					break;
				default:
					panic();
			}
			
			brewTimes[currentBrew] = brewTimes[currentBrew] - 1;
			
			currentBrew = (currentBrew + 1) % 4;
			vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
			if (brewTimes[currentBrew] == 0)
			{
				STM_EVAL_LEDOff(currentBrew);
				//Sound here
			}
		}
	}
}

int main(void)
{	
	blueCmd =   (int*)pvPortMalloc(sizeof(int));
	greenCmd =  (int*)pvPortMalloc(sizeof(int));
	orangeCmd = (int*)pvPortMalloc(sizeof(int));
	redCmd = 	(int*)pvPortMalloc(sizeof(int));
	offCmd = 	(int*)pvPortMalloc(sizeof(int));
	onCmd = 	(int*)pvPortMalloc(sizeof(int));
	brewToggle = (int*)pvPortMalloc(sizeof(int));
	*offCmd = 0;
	*onCmd = 1;
	*brewToggle = 1;
		
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); //Enable preemption. Must happen before scheduler.
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	
	xBlueQueue 		= xQueueCreate(5, sizeof(int));
	xGreenQueue 	= xQueueCreate(5, sizeof(int));
	xOrangeQueue 	= xQueueCreate(5, sizeof(int));
	xRedQueue 		= xQueueCreate(5, sizeof(int));
		
	xTaskCreate( vCoffeeThread, "CoffeeThread", 	STACK_SIZE_MIN, (void*)0, 	 1, NULL );
	
	clickTimer = xTimerCreate("Double click timer", pdMS_TO_TICKS(750), pdFALSE, (void*)0, vClickTimerHandler);
	STM_EVAL_LEDOn(LED_BLUE);
	
	vTaskStartScheduler();
	panic();
}
