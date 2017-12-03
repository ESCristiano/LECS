#include "C3DLedMatrixBuffer.h"

void C3DLedMatrixBuffer::pushFrame(C3DLedMatrix)
{
}

C3DLedMatrix C3DLedMatrixBuffer::popFrameAndWrite()
{
	return C3DLedMatrix();
}

C3DLedMatrixBuffer* C3DLedMatrixBuffer::instance = 0;

C3DLedMatrixBuffer* C3DLedMatrixBuffer::getInstance()
{
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton
	if (instance == 0)
		instance = new C3DLedMatrixBuffer;
	//mutex unclock 
	return instance;
}

C3DLedMatrixBuffer::C3DLedMatrixBuffer()
{
}


C3DLedMatrixBuffer::~C3DLedMatrixBuffer()
{
}
