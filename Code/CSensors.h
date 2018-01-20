#ifndef _CSENSORS_H
#define _CSENSORS_H

#include <stm32f4xx.h>

class CSensors {
private:
	struct dataSensors {
		char ldrBrightness;
		int microphone;
		uint8_t capsensors;
	}data;
	char t;
	static CSensors* instance;
	CSensors();
	~CSensors();
public:
	void setDataLdr( char);
	void setDataCapSensors( uint8_t );
	void setDataMicrophone( int);
	char getDataLdr();
	uint8_t getDataCapSensors();
	int getDataMicrophone();
	void initSensors();
	static CSensors* getInstance();

};

#endif //_CSENSORS_H
