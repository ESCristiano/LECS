#include "CSensors.h"
#include "CCapSensors.h"
#include "CMicrophone.h"
#include "CLightSensor.h"


CSensors::CSensors()
{
}


CSensors::~CSensors()
{
}

/*
		The brightness scale is split in 8.
													ADC value (LDR) 
		brightness level 0 	-> 	[0 to 32]
		brightness level 1 	-> 	[33 to 64] 
		brightness level 2 	-> 	[65 to 96]
		brightness level 3 	-> 	[97 to 128]
		brightness level 4 	-> 	[129 to 160]
		brightness level 5 	-> 	[161 to 192]
		brightness level 6 	-> 	[193 to 224]
		brightness level 7 	-> 	[225 to 255]
*/
void CSensors::setDataLdr(char brightness)
{
	data.ldrBrightness = brightness;	t = 2;
}

void CSensors::setDataCapSensors(int cap)
{
	data.capsensors = cap;
}

void CSensors::setDataMicrophone(int micro)
{
	data.microphone = micro;
}

char CSensors::getDataLdr()
{

	return data.ldrBrightness;
}

int CSensors::getDataCapSensors()
{
	return data.capsensors;
}

int CSensors::getDataMicrophone()
{
	return data.microphone;
}

void CSensors::initSensors()
{
	CCapSensors* cap = CCapSensors::getInstance();
	CMicrophone* micro = CMicrophone::getInstance();
	CLightSensor* ldr = CLightSensor::getInstance();

	cap->initCapacitiveSensor();
	micro->initMicrophone();
	ldr->initLightSensor();
	
}

CSensors* CSensors::instance = 0;

CSensors * CSensors::getInstance()
{
//		extern SemaphoreHandle_t mutexSensors;
	
		/*Lock Mutex*/
//			if( xSemaphoreTake( mutexSensors, ( TickType_t ) 10 ) == pdTRUE )
//        {
    if (instance == 0)
			instance = new CSensors;
//        }
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton

	//mutex unclock 
	return instance;
}
