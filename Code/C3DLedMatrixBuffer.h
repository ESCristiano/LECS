#ifndef _C3DLEDMATRIXBUFFER_H
#define _C3DLEDMATRIXBUFFER_H

#include <queue>
#include "C3DLedMatrix.h"

using namespace std;

class C3DLedMatrixBuffer {
public:
	void pushFrame(C3DLedMatrix);
	C3DLedMatrix popFrameAndWrite();
	static C3DLedMatrixBuffer* getInstance();
private:
	queue<C3DLedMatrix> buffer3DLedMatrixFrames;
	static C3DLedMatrixBuffer* instance;
	C3DLedMatrixBuffer();
	~C3DLedMatrixBuffer();
};

#endif //_C3DLEDMATRIXBUFFER_H
