#ifndef _CSENSORS_H
#define _CSENSORS_H

class CSensors {
private:
	struct dataSensors {
		char ldrBrightness;
		int microphone;
		int capsensors;
	}data;
	char t;
	static CSensors* instance;
	CSensors();
	~CSensors();
public:
	void setDataLdr( char);
	void setDataCapSensors( int);
	void setDataMicrophone( int);
	char getDataLdr();
	int getDataCapSensors();
	int getDataMicrophone();
	void initSensors();
	static CSensors* getInstance();

};

#endif //_CSENSORS_H
