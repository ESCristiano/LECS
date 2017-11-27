/*
 ****************************************************** LECS ********************************************************
 * by Cristiano Rodrigues and Ivo Marques
 *
 * Project features:
 *
 * - Adapt the effects to the environment based on data received by the sensors using STM32F4 Discovery;
 * - Produce 3D effects in a matrix of leds;
 * - Recognize some gestures;
 * - Recognize diferent levels of luminosity;
 * - Record sounds of the environment;
 * - Detect low and high frequencies (bass and treble) based in analyses the sound captured by microphone;
 * - Standby mode to save energy (when the luminosity is too high, LEDS power off).
 *
 ********************************************************************************************************************/

/* Scheduler includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* Library includes. */
#include <stm32f4xx.h>

/* Our includes*/
#include "main.h"
#include "CLdrSensor.h"

void prvSetupLed(void);

uint16_t value;

void vLEDTask( void *pvParameters )
{
	prvSetupLed();
	
	CLdrSensor ldr;
	
	ldr.initLDR();
	ldr.initTimer();
	
	for( ;; )
	{
		if(value >200)
		{
			leds.resetBlue();
			leds.resetGreen();
			leds.resetRed();
			leds.setOrange();
		}			
		else
			if(value >150)
			{
				leds.resetBlue();
				leds.resetGreen();
				leds.resetOrange();
				leds.setRed();
			
			}				
			else
				if(value >100)
				{
					leds.resetRed();
					leds.resetGreen();
					leds.resetOrange();
					leds.setBlue();	
				}
				else
					if(value >= 0)
					{	
					leds.resetRed();
					leds.resetBlue();
					leds.resetOrange();
					leds.setGreen();	
					}
	}
}



int main()
{
	
	portBASE_TYPE task1_pass;
	
	/* Create Task */
	task1_pass = xTaskCreate( vLEDTask, "Task_Led", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	
	if( task1_pass == pdPASS )
	{
			/* Start the Scheduler */ 
			vTaskStartScheduler(); 
	}
	else
	{
			/* ERROR! Creating the Tasks */
			return -2;
	}
	
	return 0;
}


void prvSetupLed(void)
{
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct;
	// Enabling GPIO peripheral clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// GPIO peripheral properties specification
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; // LED3 GPIO pin
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}
