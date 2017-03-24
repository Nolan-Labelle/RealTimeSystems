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

void panic() //If something has gone horribly wrong, then disco mode.
{
	STM_EVAL_LEDOn(LED_BLUE);
	STM_EVAL_LEDOn(LED_GREEN);
	STM_EVAL_LEDOn(LED_ORANGE);
	STM_EVAL_LEDOn(LED_RED);
}

void vBlueCoffee(void *pvParameters)
{	
	for(;;)
	{
		while(xQueueReceive(xBlueQueue, blueCmd, 10))
		{
			if (*blueCmd == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_BLUE);
			}
			else if (*blueCmd == LED_ON)
			{
				STM_EVAL_LEDOn(LED_BLUE);
			}
			else
			{
				panic();
			}
		}
	}
}

void vGreenCoffee(void *pvParameters)
{	
	for(;;)
	{
		while(xQueueReceive(xGreenQueue, greenCmd, 10))
		{
			if (*greenCmd == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_GREEN);
			}
			else if (*greenCmd == LED_ON)
			{
				STM_EVAL_LEDOn(LED_GREEN);
			}
			else
			{
				panic();
			}
		}
	}
}

void vOrangeCoffee(void *pvParameters)
{	
	for(;;)
	{
		while(xQueueReceive(xOrangeQueue, orangeCmd, 10))
		{
			if (*orangeCmd == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_ORANGE);
			}
			else if (*orangeCmd == LED_ON)
			{
				STM_EVAL_LEDOn(LED_ORANGE);
			}
			else
			{
				panic();
			}
		}
	}
}

void vRedCoffee(void *pvParameters)
{	
	for(;;)
	{
		while(xQueueReceive(xRedQueue, redCmd, 10))
		{
			if (*redCmd == LED_OFF)
			{
				STM_EVAL_LEDOff(LED_RED);
			}
			else if (*redCmd == LED_ON)
			{
				STM_EVAL_LEDOn(LED_RED);
			}
			else
			{
				panic();
			}
		}
	}
}

void singleClick()
{
	//move LED around in a circle, send messages to all 4 color queues
	//will probably break later when we need multiple LED's on/blinking at once.
	int fail = 0;
	
	switch (whichLED)
	{
		case LED_BLUE:
			whichLED = 	LED_GREEN;
			if(!xQueueSend(xBlueQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, onCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, offCmd, 0))
				fail++;
			break;
		case LED_GREEN:
			whichLED = 	LED_ORANGE;
			if(!xQueueSend(xBlueQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, onCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, offCmd, 0))
				fail++;
			break;
		case LED_ORANGE:
			whichLED = 	LED_RED;
			if(!xQueueSend(xBlueQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, onCmd, 0))
				fail++;
			break;
		case LED_RED:
			whichLED = 	LED_BLUE;
			if(!xQueueSend(xBlueQueue, onCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, offCmd, 0))
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

void doubleClick()
{
	int fail = 0;
	
	switch (whichLED)
	{
		case LED_BLUE:
			whichLED = 	LED_RED;
			if(!xQueueSend(xBlueQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, onCmd, 0))
				fail++;
			break;
		case LED_GREEN:
			whichLED = 	LED_BLUE;
			if(!xQueueSend(xBlueQueue, onCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, offCmd, 0))
				fail++;
			break;
		case LED_ORANGE:
			whichLED = 	LED_GREEN;
			if(!xQueueSend(xBlueQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, onCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, offCmd, 0))
				fail++;
			break;
		case LED_RED:
			whichLED = 	LED_ORANGE;
			if(!xQueueSend(xBlueQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xGreenQueue, offCmd, 0))
				fail++;
			if(!xQueueSend(xOrangeQueue, onCmd, 0))
				fail++;
			if(!xQueueSend(xRedQueue, offCmd, 0))
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

void vSelector(void *pvParameters)
{	
	for(;;)
	{
		//wait for button?
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
	*offCmd = 0;
	*onCmd = 1;
	
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
		
	xTaskCreate( vBlueCoffee, 	"Blue coffee", 		STACK_SIZE_MIN, (void*)1000, 1, NULL );
	xTaskCreate( vGreenCoffee, 	"Green coffee", 	STACK_SIZE_MIN, (void*)2000, 1, NULL );
	xTaskCreate( vOrangeCoffee, "Orange coffee", 	STACK_SIZE_MIN, (void*)4000, 1, NULL );
	xTaskCreate( vRedCoffee, 	"Red coffee", 		STACK_SIZE_MIN, (void*)8000, 1, NULL );
	xTaskCreate( vSelector, 	"Selector", 		STACK_SIZE_MIN, (void*)0, 	 1, NULL );
	
	clickTimer = xTimerCreate("Double click timer", pdMS_TO_TICKS(750), pdFALSE, (void*)0, vClickTimerHandler);
	STM_EVAL_LEDOn(LED_BLUE);
	
	vTaskStartScheduler();
	panic();
}
