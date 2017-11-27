#ifndef __INITIALIZATIONS_H__
#define __INITIALIZATIONS_H__

/* 
 *
 * This module is for do all peripherals initialization
 *
 */
 
 /* Scheduler includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* Library includes. */
#include <stm32f4xx.h>

/*-------------Includes ---------------*/
#include "CLdrSensor.h"
#include "CBoardLeds.h"

class CInitializations
{
public:
	CInitializations();
	~CInitializations();
	void init3DLedMatrix();
	void initCapacitiveSensor();
	void initMicrophone();
	void initLightSensor();
	int initTasks();
};

#endif // __INITIALIZATIONS_H__
