#ifndef _CMICROPHONE_H
#define _CMICROPHONE_H

#include <vector>

using namespace std;

class CMicrophone {
public:
	void initMicrophone();
	void closeMicrophone();
	int readMicrophone();

	static CMicrophone* getInstance();
private:
	static CMicrophone* instance;
	vector<int> soundWave;
	CMicrophone();
	~CMicrophone();
};

#endif //_CMICROPHONE_H

