#include "CQueue.h"

CQueue::CQueue():
		read(0),
		write(0),
		capacity(__FRAMES)
{
	mutexPush = xSemaphoreCreateMutex();
	mutexPop = xSemaphoreCreateMutex();
	
	for(int i=0 ; i<__FRAMES; i++)
	{
		data[i] = new C3DLedMatrix;
	}
}
	

CQueue::~CQueue()
{
	
	for(int i=0 ; i<__FRAMES; i++)
	{
		delete[] data[i];
	}
}

/*******************************************************************************
* Function Name  : push
* Description    : put an element in the queue
* Input          : const C3DLedMatrix *t -> element 
* Output         : None 
* Return			   : None
*******************************************************************************/
void CQueue::push(const C3DLedMatrix *t)
{
	/*Lock Mutex*/
	if( xSemaphoreTake( mutexPush, ( TickType_t ) 10 ) == pdTRUE )
	{
		if (size() >= __FRAMES)
			return;//error
		*data[write & (__FRAMES - 1)] = *t;
		write++;
		/*Unlock Mutex*/
		xSemaphoreGive( mutexPush );
	}
	
}

/*******************************************************************************
* Function Name  : pop
* Description    : remove an element in the queue
* Input          : None 
* Output         : C3DLedMatrix 
* Return			   : element type C3DLedMatrix
*******************************************************************************/
C3DLedMatrix * CQueue::pop()
{
	static C3DLedMatrix *aux = new C3DLedMatrix, *lastState = new C3DLedMatrix;
//	C3DLedMatrix *auxDestructor = new C3DLedMatrix;
	
	/*Lock Mutex*/
	if( xSemaphoreTake( mutexPop, ( TickType_t ) 10 ) == pdTRUE )
	{
	if (!size()) //when all element was read
	{
		/*Unlock Mutex*/
		xSemaphoreGive( mutexPop );
		return lastState; //alast element is returned
	}
	
	*aux = *data[read++ & (__FRAMES-1)];
	*lastState = *aux;

//  data[read++ & (__FRAMES-1)]->~C3DLedMatrix(); // acho que falta este destrutor....programa ao final de um tempo crasha
	
	/*Unlock Mutex*/
	xSemaphoreGive( mutexPop );
	return aux;
	}
}
