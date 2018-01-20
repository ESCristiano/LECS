#ifndef _CCAPSENSORS_H
#define _CCAPSENSORS_H

#include "freertos.h"

class CCapSensors {
public:
	void initCapacitiveSensor();
	void closeCapacitiveSensor();
	uint8_t readCapSensors();
	CCapSensors static* getInstance();
private:
	CCapSensors static* instance;
	CCapSensors();
	~CCapSensors();
};

#endif //_CCAPSENSORS_H

