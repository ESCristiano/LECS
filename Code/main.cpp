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
#include "CInitializations.h"


uint16_t value;

int main()
{
	CInitializations LECS;
	
	LECS.init3DLedMatrix();
	LECS.initCapacitiveSensor();
	LECS.initLightSensor();
	LECS.initMicrophone();
	
	if( !LECS.initTasks() )
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
