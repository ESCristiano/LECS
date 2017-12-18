#include "C3DLedMatrixBuffer.h"

C3DLedMatrixBuffer::C3DLedMatrixBuffer()
{
}


C3DLedMatrixBuffer::~C3DLedMatrixBuffer()
{
}

/*******************************************************************************
* Function Name  : pushFrame
* Description    : put one frame of the C3DLedMatrix in the buffer
* Input          : C3DLedMatrix _3DMatrix, frame 
* Output         : None 
* Return			   : None
*******************************************************************************/
void C3DLedMatrixBuffer::pushFrame(C3DLedMatrix *_3DMatrix)
{
	buffer3DLedMatrixFrames.push(_3DMatrix);

}

/*******************************************************************************
* Function Name  : popFrame
* Description    : remove one frame of the C3DLedMatrix of the buffer
* Input          : None 
* Output         : C3DLedMatrix 
* Return			   : C3DLedMatrix _3DMatrix, frame
*******************************************************************************/
C3DLedMatrix *C3DLedMatrixBuffer::popFrame()
{
	return (buffer3DLedMatrixFrames.pop());
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

