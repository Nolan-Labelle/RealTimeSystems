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
#include "main.h"

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

//Globals:
fir_8 filt;
TaskHandle_t process[4];
TaskHandle_t schedHandle;
TimerHandle_t schedulerTimer;

void panic() //If something has gone horribly wrong, then disco mode.
{
	STM_EVAL_LEDOn(LED_BLUE);
	STM_EVAL_LEDOn(LED_GREEN);
	STM_EVAL_LEDOn(LED_ORANGE);
	STM_EVAL_LEDOn(LED_RED);
}

void EXTI0_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EXTI_ClearITPendingBit(EXTI_Line0);
	if(xTimerStartFromISR(schedulerTimer, &xHigherPriorityTaskWoken) != pdPASS)
	{
		//Timer was not able to start
		panic();
	}
	if(xHigherPriorityTaskWoken != pdFALSE)
	{
		panic();
	}
}

void beep()
{
	if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
 	{
 		int counter = 4150000;
		while(counter--) //code gotten from the sample sound program that was provided to save time upon creating the sine wave
    {
    	if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
    	{
    		SPI_I2S_SendData(CODEC_I2S, sample);

    		//only update on every second sample to insure that L & R ch. have the same sample value
    		if (sampleCounter & 0x00000001)
    		{
    			sawWave += NOTEFREQUENCY;
    			if (sawWave > 1.0)
    				sawWave -= 2.0;

    			filteredSaw = updateFilter(&filt, sawWave);
    			sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
    		}
    		sampleCounter++;
    	}

    	else if (sampleCounter == 96000)
    	{
    		sampleCounter = 0;
    	}
		}
  }
}
float updateFilter(fir_8* filt, float val)
{
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter)
{
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}

void vTimer (TimerHandle_t xTimer)
{
	panic();
	//vTaskResume(&schedHandle);
}
void vSchedTask (void* pvParameters)
{
	//xTimerStart(schedulerTimer, 0);
	//vTaskPrioritySet(&schedHandle, 1);
	for(;;)
	{
		#ifdef FPS
			//panic();
		#endif
		#ifdef EDF
			//
		#endif
		#ifdef LLF
		//rTimer,
		#endif
	}
}
void vBlueThread (void* pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDOn(LED_BLUE);
		STM_EVAL_LEDOff(LED_GREEN);
		STM_EVAL_LEDOff(LED_ORANGE);
		STM_EVAL_LEDOff(LED_RED);
	}
}
void vGreenThread (void* pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDOff(LED_BLUE);
		STM_EVAL_LEDOn(LED_GREEN);
		STM_EVAL_LEDOff(LED_ORANGE);
		STM_EVAL_LEDOff(LED_RED);
	}
}
void vOrangeThread (void* pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDOff(LED_BLUE);
		STM_EVAL_LEDOff(LED_GREEN);
		STM_EVAL_LEDOn(LED_ORANGE);
		STM_EVAL_LEDOff(LED_RED);
	}
}
void vRedThread (void* pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDOff(LED_BLUE);
		STM_EVAL_LEDOff(LED_GREEN);
		STM_EVAL_LEDOff(LED_ORANGE);
		STM_EVAL_LEDOn(LED_RED);
	}
}

int main(void)
{	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); //Enable preemption. Must happen before scheduler.
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	codec_init();
	codec_ctrl_init();
	I2S_Cmd(CODEC_I2S, ENABLE);
	initFilter(&filt);
		
	xTaskCreate( vSchedTask, 		"Scheduler Task", STACK_SIZE_MIN, (void*)0, 	 2, &schedHandle);
	//xTaskCreate( vBlueThread, 	"BlueThread", 		STACK_SIZE_MIN, (void*)0, 	 1, &process[0] );
	//xTaskCreate( vGreenThread, 	"GreenThread", 		STACK_SIZE_MIN, (void*)0, 	 1, &process[1] );
	//xTaskCreate( vOrangeThread, "OrangeThread", 	STACK_SIZE_MIN, (void*)0, 	 1, &process[2] );
	//xTaskCreate( vRedThread, 		"RedThread", 			STACK_SIZE_MIN, (void*)0, 	 1, &process[3] );
	
	schedulerTimer = xTimerCreate("Scheduler timer", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, vTimer);
	
	vTaskStartScheduler();
	panic();
}
