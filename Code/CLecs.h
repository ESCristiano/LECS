#ifndef _CLECS_H
#define _CLECS_H

#include "freertos.h"

/* Library includes. */
#include <stm32f4xx.h> 
#include "CLayer.h"

using namespace std;

class CLecs {
public:
	void initNVIC();
	void init3DLedMatrix();
	void initLecsSensors();
	void initSemaphores();
	void initQueue();
	int run();
	static CLecs* getInstance();
private:
	int initTasks();
	static CLecs* instance;
	CLecs();
	~CLecs();
};

#endif //_CLECS_H
