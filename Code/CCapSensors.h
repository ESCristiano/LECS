#ifndef _CCAPSENSORS_H
#define _CCAPSENSORS_H

class CCapSensors {
public:
	void initCapacitiveSensor();
	void closeCapacitiveSensor();
	int readCapSensors();
	CCapSensors static* getInstance();
private:
	CCapSensors static* instance;
	CCapSensors();
	~CCapSensors();
};

#endif //_CCAPSENSORS_H

