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
#define NUMPROCS 4
#define FPS

typedef enum
{
	BLUE 	= 0,
	GREEN 	= 1,
	ORANGE 	= 2,
	RED 	= 3
} procEnum;

typedef enum
{
	RUNNABLE = 0,
	RUNNING = 1,
	BLOCKED = 2
} state;

typedef struct
{
	state runState;
	int deadline;
	int period;
	int hasRun;
	int duration;
	int priority;
	TaskHandle_t handler;
} proc;

//Globals:
fir_8 filt;
static TaskHandle_t schedHandle;
static TimerHandle_t schedulerTimer;
static int time;
static int brewing = 0; //boolean
static proc process[NUMPROCS]; //Process table. Need locks?

void panic()
{
	STM_EVAL_LEDOn(LED_BLUE);
	STM_EVAL_LEDOn(LED_GREEN);
	STM_EVAL_LEDOn(LED_ORANGE);
	STM_EVAL_LEDOn(LED_RED);
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

void EXTI0_IRQHandler()
{
	int i;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(xTimerStartFromISR(schedulerTimer, &xHigherPriorityTaskWoken) != pdPASS)
	{
		//Timer was not able to start
		panic();
	}
	brewing = !brewing;
	time = 0;
	
	//Reset proc stats
	for(i = 0; i < NUMPROCS; i++) 
	{
		process[i].runState = RUNNABLE;
		process[i].hasRun = 0;
	}
	
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void vTimerHandler (TimerHandle_t xTimer)
{
	int i;
	time++;
	
	if(brewing)
	{
		//Check if a proc's period has come up. If it has, ready the proc for running.
		for(i = 0; i < NUMPROCS; i++)
		{
			if(time % process[i].period == 0)
			{
				process[i].runState = RUNNABLE;
				process[i].hasRun = 0;
			}
		}
		
		//Implicit scheduler call
		for(i = 0; i < NUMPROCS; i++)
		{
			vTaskPrioritySet(process[i].handler, 1);
		}
	}
}

//Written as if deadlines don't matter in fixed priority.
void fixedPriorityScheduler()
{
	int i, currentMaxPriority = 0;
	proc *selectedProcess = 0;
	
	for(i = 0; i < NUMPROCS; i++) //Choose proc to run
	{
		if((process[i].runState == RUNNABLE) && (process[i].priority > currentMaxPriority))
		{
			selectedProcess = &process[i];
			currentMaxPriority = selectedProcess->priority;
		}
	}
	
	if(selectedProcess != 0) //If we've chosen a proc to run
	{
		selectedProcess->hasRun += 1;
		selectedProcess->runState = RUNNING;
		vTaskPrioritySet(selectedProcess->handler, 3); //implicit exit of scheduler		
		selectedProcess->runState = RUNNABLE; //Proc has finished running when control flow gets back here
		if(selectedProcess->hasRun == selectedProcess->duration) //Block if we've brewed an entire duration's worth
		{
			selectedProcess->runState = BLOCKED;
		}
	}
	else
	{
		STM_EVAL_LEDOff(LED_BLUE);
		STM_EVAL_LEDOff(LED_GREEN);
		STM_EVAL_LEDOff(LED_ORANGE);
		STM_EVAL_LEDOff(LED_RED);
	}
}

void vSchedTask (void* pvParameters)
{
	int count = 0;

	for(;;)
	{
		if(brewing)
		{
			#ifdef FPS
				fixedPriorityScheduler();
			#endif
			#ifdef EDF
				panic();
			#endif
			#ifdef LLF
				panic();
			#endif
			#ifdef SPINDEMO
				vTaskPrioritySet(process[count%NUMPROCS].handler, 3);
				count++; //Wow this one's small. Code golf!
			#endif
		}
		else
		{
			STM_EVAL_LEDOff(LED_BLUE);
			STM_EVAL_LEDOff(LED_GREEN);
			STM_EVAL_LEDOff(LED_ORANGE);
			STM_EVAL_LEDOff(LED_RED);
		}
	}
}

int main(void)
{	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	codec_init();
	codec_ctrl_init();
	I2S_Cmd(CODEC_I2S, ENABLE);
	initFilter(&filt);
	
	process[BLUE].runState = RUNNABLE;
	process[BLUE].deadline = 5;
	process[BLUE].period = 20;
	process[BLUE].hasRun = 0;
	process[BLUE].priority = 3;
	process[BLUE].duration = 3;
	
	process[GREEN].runState = RUNNABLE;
	process[GREEN].deadline = 10;
	process[GREEN].period = 30;
	process[GREEN].hasRun = 0;
	process[GREEN].priority = 1;
	process[GREEN].duration = 4;

	process[ORANGE].runState = RUNNABLE;
	process[ORANGE].deadline = 10;
	process[ORANGE].period = 40;
	process[ORANGE].hasRun = 0;
	process[ORANGE].priority = 2;
	process[ORANGE].duration = 4;

	process[RED].runState = RUNNABLE;
	process[RED].deadline = 15;
	process[RED].period = 40;
	process[RED].hasRun = 0;
	process[RED].priority = 2;
	process[RED].duration = 6;
	
	xTaskCreate( vSchedTask, 	"Scheduler Task", 	STACK_SIZE_MIN, (void*)0, 	 2, &schedHandle);
	xTaskCreate( vBlueThread, 	"BlueThread", 		STACK_SIZE_MIN, (void*)0, 	 1, &process[BLUE].handler );
	xTaskCreate( vGreenThread, 	"GreenThread", 		STACK_SIZE_MIN, (void*)0, 	 1, &process[GREEN].handler );
	xTaskCreate( vOrangeThread, "OrangeThread", 	STACK_SIZE_MIN, (void*)0, 	 1, &process[ORANGE].handler );
	xTaskCreate( vRedThread, 	"RedThread", 		STACK_SIZE_MIN, (void*)0, 	 1, &process[RED].handler );
	
	schedulerTimer = xTimerCreate("Scheduler timer", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, vTimerHandler);
	
	vTaskStartScheduler();
	panic();
}
