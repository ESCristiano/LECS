#include "CCapSensors.h"

CCapSensors::CCapSensors()
{
}


CCapSensors::~CCapSensors()
{
}

void CCapSensors::initCapacitiveSensor()
{
}

void CCapSensors::closeCapacitiveSensor()
{
}

int CCapSensors::readCapSensors()
{
	return 0;
}

CCapSensors* CCapSensors::instance = 0;

CCapSensors * CCapSensors::getInstance()
{
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton
	if (instance == 0)
		instance = new CCapSensors;
	//mutex unclock 
	return instance;
}

