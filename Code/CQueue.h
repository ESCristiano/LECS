#ifndef	_CQUEUE_H
#define _CQUEUE_H

#include "freertos.h"
#include "C3DLedMatrix.h"


#define __FRAMES 32

class CQueue {
public:
	CQueue();
	~CQueue(); // release the CQueue:

	void push(const C3DLedMatrix *t);
	C3DLedMatrix* pop();// remove oldest object from queue:

private:
	C3DLedMatrix *data[__FRAMES];
	unsigned int read;
	unsigned int write;
	const int capacity;
	SemaphoreHandle_t mutexPush, mutexPop;
	int size() 

	{	if(write - read > (__FRAMES-1))
			read = write - (__FRAMES - 1);
		return (write - read); }
};



#endif //_CQUEUE_H
