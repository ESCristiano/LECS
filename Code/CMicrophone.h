#ifndef _CMICROPHONE_H
#define _CMICROPHONE_H

#define __FPU_PRESENT   						1
#define ARM_MATH_CM4 								//indicate lib arm_math.h that we are using a coreM4
#include "freertos.h"
#include "arm_math.h"
#include "arm_const_structs.h"


class CMicrophone {
public:
	void initMicrophone();
	void closeMicrophone();
	int readMicrophone();

	static CMicrophone* getInstance();
private:
	static CMicrophone* instance;
	int soundWave;
	void GPIOInit(void);
	void SPIInit(uint32_t Freq);
	void NVICInit(void);
	uint32_t waveRecorderInit(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
	uint8_t waveRecorderStart(uint16_t* pbuf, uint32_t size);
	uint32_t waveRecorderStop(void);
	uint16_t maxArray(uint16_t a[], uint16_t num_elements);
	void waveRecorderUpdate(void);

	CMicrophone();
	~CMicrophone();
};

#endif //_CMICROPHONE_H

