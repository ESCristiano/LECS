#ifndef _CTIMER_H
#define _CTIMER_H

#include <misc.h>

using namespace std;

class CTimer {
public:
	CTimer();
	~CTimer();
	void timerInterruptInit( uint8_t);
	void timerInit( uint16_t,  uint16_t,  uint32_t,  TIM_TypeDef*);
	void timerStart( TIM_TypeDef*);
	void timerStop( TIM_TypeDef*);
	void timerInterruptEnable( TIM_TypeDef*);
	void timerInterruptDisable( TIM_TypeDef*);
};

#endif //_CTIMER_H


