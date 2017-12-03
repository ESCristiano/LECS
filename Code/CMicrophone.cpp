#include "CMicrophone.h"

CMicrophone::CMicrophone()
{
}


CMicrophone::~CMicrophone()
{
}

void CMicrophone::initMicrophone()
{
}

void CMicrophone::closeMicrophone()
{
}

int CMicrophone::readMicrophone()
{
	return 0;
}

CMicrophone* CMicrophone::instance = 0;

CMicrophone * CMicrophone::getInstance()
{
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton
	if (instance == 0)
		instance = new CMicrophone;
	//mutex unclock 
	return instance;
}

