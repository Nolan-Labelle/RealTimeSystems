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
#define LED_OFF 0
#define LED_ON  1

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
static QueueHandle_t xBlueQueue;
static QueueHandle_t xGreenQueue;
static QueueHandle_t xOrangeQueue;
static QueueHandle_t xRedQueue;
static TimerHandle_t clickTimer;
static int lastPress = 0;
static int debounceDelay = 500 / portTICK_RATE_MS;
static TickType_t thisPress;
static int p = 0;

void panic() //If something has gone horribly wrong, then disco mode.
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_BLUE);
		STM_EVAL_LEDToggle(LED_GREEN);
		STM_EVAL_LEDToggle(LED_ORANGE);
		STM_EVAL_LEDToggle(LED_RED);
		vTaskDelay(250 / portTICK_RATE_MS);
	}
}

void vBlueCoffee(void *pvParameters)
{
	int sentCommand;
	
	for(;;)
	{
		while(xQueueReceive(xBlueQueue, &sentCommand, 0))
		{
			if (sentCommand == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_BLUE);
			}
			else if (sentCommand == LED_ON)
			{
				STM_EVAL_LEDOn(LED_BLUE);
			}
		}
	}
}

void vGreenCoffee(void *pvParameters)
{
	int sentCommand;
	
	for(;;)
	{
		while(xQueueReceive(xGreenQueue, &sentCommand, 0))
		{
			if (sentCommand == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_GREEN);
			}
			else if (sentCommand == LED_ON)
			{
				STM_EVAL_LEDOn(LED_GREEN);
			}
		}
	}
}

void vOrangeCoffee(void *pvParameters)
{
	int sentCommand;
	
	for(;;)
	{
		while(xQueueReceive(xOrangeQueue, &sentCommand, 0))
		{
			if (sentCommand == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_ORANGE);
			}
			else if (sentCommand == LED_ON)
			{
				STM_EVAL_LEDOn(LED_ORANGE);
			}
		}
	}
}

void vRedCoffee(void *pvParameters)
{
	int sentCommand;
	
	for(;;)
	{
		while(xQueueReceive(xRedQueue, &sentCommand, 0))
		{
			if (sentCommand == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_RED);
			}
			else if (sentCommand == LED_ON)
			{
				STM_EVAL_LEDOn(LED_RED);
			}
		}
	}
}

void singleClick()
{
	//move LED around in a circle, send messages to all 4 color queues
	//will probably break later when we need multiple LED's on/blinking at once.
	switch (whichLED)
	{
		case LED_BLUE:
			whichLED = LED_GREEN;
			xQueueSendFromISR(xBlueQueue, 	LED_OFF, NULL);
			xQueueSendFromISR(xGreenQueue, 	(void*)LED_ON, NULL);
			xQueueSendFromISR(xOrangeQueue, LED_OFF, NULL);
			xQueueSendFromISR(xRedQueue, 	LED_OFF, NULL);
			break;
		case LED_GREEN:
			whichLED = LED_ORANGE;
			xQueueSendFromISR(xBlueQueue, 	LED_OFF, NULL);
			xQueueSendFromISR(xGreenQueue, 	LED_OFF, NULL);
			xQueueSendFromISR(xOrangeQueue, (void*)LED_ON, NULL);
			xQueueSendFromISR(xRedQueue, 	LED_OFF, NULL);
			break;
		case LED_ORANGE:
			whichLED = LED_RED;
			xQueueSendFromISR(xBlueQueue, 	LED_OFF, NULL);
			xQueueSendFromISR(xGreenQueue, 	LED_OFF, NULL);
			xQueueSendFromISR(xOrangeQueue, LED_OFF, NULL);
			xQueueSendFromISR(xRedQueue, 	(void*)LED_ON, NULL);
			break;
		case LED_RED:
			whichLED = LED_BLUE;
			xQueueSendFromISR(xBlueQueue, 	(void*)LED_ON,  NULL);
			xQueueSendFromISR(xGreenQueue, 	LED_OFF, NULL);
			xQueueSendFromISR(xOrangeQueue, LED_OFF, NULL);
			xQueueSendFromISR(xRedQueue, 	LED_OFF, NULL);
			break;
		default:
			panic();
	}
}

void doubleClick()
{
	//nothing yet.
}

void vClickTimerHandler(TimerHandle_t xTimer)
{
	STM_EVAL_LEDToggle(LED_RED);
	if (clicks == 1)
		singleClick();
	if (clicks >= 2)
		doubleClick();
	if (clicks <= 0)
		panic();
}

void EXTI0_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	thisPress = xTaskGetTickCountFromISR();
	if((thisPress - lastPress) > debounceDelay)
	{
		if (clicks == 0)
		{
			if(xTimerStartFromISR(clickTimer, &xHigherPriorityTaskWoken) != pdPASS)
			{
				//Timer was not able to start
			}
		}
		clicks++;
	}
	lastPress = thisPress;
	
	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		STM_EVAL_LEDToggle(LED_ORANGE);
		//portYIELD_FROM_ISR(1);
		EXTI_ClearITPendingBit(EXTI_Line0); //Holy fuck that took all night.
	}
}

void vSelector(void *pvParameters)
{	
	for(;;)
	{
		//wait for button?
	}
}

int main(void)
{
	SelectorPack* selectorPack = (SelectorPack*)pvPortMalloc(sizeof(SelectorPack));
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); //Enable preemption. Must happen before scheduler.
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	
	xBlueQueue = 	xQueueCreate(5, sizeof(int));
	xGreenQueue = 	xQueueCreate(5, sizeof(int));
	xOrangeQueue = 	xQueueCreate(5, sizeof(int));
	xRedQueue = 	xQueueCreate(5, sizeof(int));
	
	STM_EVAL_LEDOn(LED_BLUE); //Start with the blue LED on
	STM_EVAL_LEDOff(LED_GREEN);
	STM_EVAL_LEDOff(LED_ORANGE);
	STM_EVAL_LEDOff(LED_RED);
		
	xTaskCreate( vBlueCoffee, 	"Blue coffee", 		STACK_SIZE_MIN, (void*)1000, 	 1, NULL );
	xTaskCreate( vGreenCoffee, 	"Green coffee", 	STACK_SIZE_MIN, (void*)2000, 	 1, NULL );
	xTaskCreate( vOrangeCoffee, "Orange coffee", 	STACK_SIZE_MIN, (void*)4000, 	 1, NULL );
	xTaskCreate( vRedCoffee, 	"Red coffee", 		STACK_SIZE_MIN, (void*)8000, 	 1, NULL );
	xTaskCreate( vSelector, 	"Selector", 		STACK_SIZE_MIN, (void*)selectorPack, 1, NULL );
	
	clickTimer = xTimerCreate("Double click timer", pdMS_TO_TICKS(500), pdFALSE, (void*)0, vClickTimerHandler);
	
	vTaskStartScheduler();
	panic();
}
