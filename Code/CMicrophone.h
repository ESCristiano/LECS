#ifndef _CMICROPHONE_H
#define _CMICROPHONE_H

class CMicrophone {
public:
	void initMicrophone();
	void closeMicrophone();
	int readMicrophone();

	static CMicrophone* getInstance();
private:
	static CMicrophone* instance;
	int soundWave;
	CMicrophone();
	~CMicrophone();
};

#endif //_CMICROPHONE_H

