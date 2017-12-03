#ifndef _CLIGHTSENSOR_H
#define _CLIGHTSENSOR_H

/*
http://www.micromouseonline.com/2016/02/03/tim3-arr-regular-interrupts-stm32f4/

Ldr is read at frequency of 1Hz

How use this module:
	1 -> initLDR(); Configure ADC and Input Pin
	2 -> initTimer(); Configure Timer Interrupt
	3 -> readLDR();
	4 -> closeLDR();
*/

/*-------------Includes ---------------*/
#include "freertos.h"
#include <stm32f4xx_adc.h>
#include "CTimer.h"

using namespace std;

class CLightSensor {
public:
	void initLightSensor();
	void closeLDR();
	uint16_t readLDR();
	static CLightSensor* getInstance();
private:
	CTimer timer7;
	uint16_t mvalueLdr; // temporária para testes
	static CLightSensor* instance;
	CLightSensor();
	~CLightSensor();
	void initLDR();
};

#endif //_CLIGHTSENSOR_H

