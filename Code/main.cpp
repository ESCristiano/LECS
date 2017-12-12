/******************************************************* LECS ********************************************************************************************************
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
 *********************************************************************************************************************************************************************/

/*Our includes*/
#include "main.h"
#include "CLecs.h"

#include <stdlib.h>
int main()
{
	CLecs* Lecs = CLecs::getInstance();
	
	Lecs->initNVIC();
	Lecs->init3DLedMatrix();
	//Lecs->initLecsSensors();
	Lecs->initSemaphores();
	Lecs->initQueue();
	Lecs->run();

	while(1);
}

/*
	Overload of opeartor new and delete because they use malloc and free
that aren not thread safe
*/
void *operator new(size_t size)
{
   void *p;

   if(uxTaskGetNumberOfTasks())
      p=pvPortMalloc(size); //thread safe
   else
      p=malloc(size); //no thread safe, just when we have only one thread

   return p;
}

void operator delete(void *p)
{
   if(uxTaskGetNumberOfTasks())
      vPortFree( p );//thread safe
   else
      free( p );//no thread safe, just when we have only one thread

   p = NULL;

}
