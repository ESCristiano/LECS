#ifndef _C3DLEDMATRIXBUFFER_H
#define _C3DLEDMATRIXBUFFER_H

#include "C3DLedMatrix.h"
#include "CQueue.h"

class C3DLedMatrixBuffer {
public:
	void pushFrame(C3DLedMatrix*);
	C3DLedMatrix* popFrame();
	static C3DLedMatrixBuffer* getInstance();
private:
	CQueue buffer3DLedMatrixFrames; 
	static C3DLedMatrixBuffer* instance;
	C3DLedMatrixBuffer();
	~C3DLedMatrixBuffer();
};

#endif //_C3DLEDMATRIXBUFFER_H
