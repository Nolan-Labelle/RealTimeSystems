#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include <stdlib.h>

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

typedef struct
{
	int brewTime;
	QueueHandle_t queuePointer;
} QueuePack;

typedef struct
{
	QueueHandle_t blue;
	QueueHandle_t red;
	QueueHandle_t green;
	QueueHandle_t orange;
} SelectorPack;

void vBlueCoffee(void *pvParameters)
{
	QueuePack* bluePack = (QueuePack*)pvParameters;
	
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_BLUE);
		vTaskDelay( 1000 / portTICK_RATE_MS );
	}
}

void vRedCoffee(void *pvParameters)
{
	QueuePack* redPack = (QueuePack*)pvParameters;
	
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_RED);
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void vGreenCoffee(void *pvParameters)
{
	QueuePack* greenPack = (QueuePack*)pvParameters;
	
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_GREEN);
		vTaskDelay( 200 / portTICK_RATE_MS );
	}
}

void vOrangeCoffee(void *pvParameters)
{
	QueuePack* orangePack = (QueuePack*)pvParameters;
	
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay( 300 / portTICK_RATE_MS );
	}
}

void vSelector(void *pvParameters)
{
	SelectorPack* selectorPack = (SelectorPack*)pvParameters;
	
	for(;;)
	{
		//lol i dunno.
	}
}

int main(void)
{
	QueueHandle_t xBlueQueue, xRedQueue, xGreenQueue, xOrangeQueue;
	QueuePack* bluePack = malloc(sizeof(QueuePack));
	QueuePack* redPack = malloc(sizeof(QueuePack));
	QueuePack* greenPack = malloc(sizeof(QueuePack));
	QueuePack* orangePack = malloc(sizeof(QueuePack));
	SelectorPack* selectorPack = malloc(sizeof(SelectorPack));
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); //Enable preemption. Must happen before scheduler.
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	
	xBlueQueue = 	xQueueCreate(10, sizeof(int));
	xRedQueue = 	xQueueCreate(10, sizeof(int));
	xGreenQueue = 	xQueueCreate(10, sizeof(int));
	xOrangeQueue = 	xQueueCreate(10, sizeof(int));
	
	xTaskCreate( vBlueCoffee, 	"Blue coffee", 		STACK_SIZE_MIN, (void*)bluePack, 1, NULL );
	xTaskCreate( vRedCoffee, 	"Red coffee", 		STACK_SIZE_MIN, (void*)redPack, 1, NULL );
	xTaskCreate( vGreenCoffee, 	"Green coffee", 	STACK_SIZE_MIN, (void*)greenPack, 1, NULL );
	xTaskCreate( vOrangeCoffee, "Orange coffee", 	STACK_SIZE_MIN, (void*)orangePack, 1, NULL );
	xTaskCreate( vSelector, 	"Selector", 		STACK_SIZE_MIN, (void*)selectorPack, 2, NULL );
	
	vTaskStartScheduler();
}
