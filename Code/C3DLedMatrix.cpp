#include "C3DLedMatrix.h"

#include "CTimer.h"
#include "C3DLedMatrixBuffer.h"

C3DLedMatrix::C3DLedMatrix()
:CLayer(), C2DLedMatrix()
{
	/*Constructors of the inherited classes*/
//	CLayer();
//	C2DLedMatrix();
	
		/*  Initialize 3D Led Matrix
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 } 
	*/
	_3Dmatrix = new char**[5];  //allocate a pointer to 2D matrixs
	for (int i = 0; i < 5; ++i)
	{
		_3Dmatrix[i] = new char*[5]; //allocate a pointer to columns
		for (int j = 0; j < 5; ++j)
			_3Dmatrix[i][j] = new char[5](); // allocate and set all elements 0
	}
}


C3DLedMatrix::~C3DLedMatrix()
{
	/*deallocate columns of the 2D matrix*/
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 5; ++j)
			delete[] _3Dmatrix[i][j]; 
	}
	
	/*deallocate rows of the 2D matrix*/
	for (int i = 0; i < 5; ++i)
	{
		delete[] _3Dmatrix[i]; 
	}
	
	/*deallocate 3D matrix*/
	delete [] _3Dmatrix; 
}


/*******************************************************************************
* Function Name  : TIM6_DAC_IRQHandler
* Description    : ISR timer 6
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
// extern "C" -> for C++, ensure the interrupt handler is linked as a C function
extern "C" void TIM6_DAC_IRQHandler(void) 
{
	static int count = 0; 
	extern SemaphoreHandle_t Sem_ISR_3D, Sem_ISR_ChangePattern;
	static BaseType_t xHigherPriorityTaskWoken;
	    xHigherPriorityTaskWoken = pdFALSE;
	
	if (TIM_GetITStatus (TIM6, TIM_IT_Update) != RESET) {

	if(count++ == 32) //30 frames por second (32ms in 32 ms change frame)
	{
		xSemaphoreGiveFromISR( Sem_ISR_ChangePattern, &xHigherPriorityTaskWoken );
		count = 0;
	}
	/* Unblock the task by releasing the semaphore. */
	xSemaphoreGiveFromISR( Sem_ISR_3D, &xHigherPriorityTaskWoken );
	TIM_ClearITPendingBit (TIM6, TIM_IT_Update); 
  }
	
}

/*******************************************************************************
* Function Name  : write3DMatrix
* Description    : write in physical 3D Matrix
*								 : This function will be called in each interrupt of the timer
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void C3DLedMatrix::write3DMatrix(void)
{
		static int layer = 0;
		layer = layer*(!(layer/5));	// variable layer is always between 0 and 4
		this->setLayer(layer);
		this->set2DMatrix(_3Dmatrix[layer]);
		this->writeLayer();
		this->write2DMatrix();
		layer++; // layer change always that occurr one call of this function
						 // Is one timer interrupt that calls this function
						 // For this reason, a layer change every interrupt of the timer
	}

/*******************************************************************************
* Function Name  : setLayer
* Description    : write in private menber _2Dmatrix
* Input          : vector< vector< vector<int> > > matrix3d, indicate the 3D 
*								 : matrix to be set 
* Output         : None 
* Return			   : None
*******************************************************************************/
void C3DLedMatrix::set3DMatrix(char ***matrix3D)
{
	/*
		copy matrix3d to _3Dmatrix
		_3Dmatrix = matrix3d
	*/
	
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++)
			for (int k = 0; k < 5; k++)
				_3Dmatrix[i][j][k] = matrix3D[i][j][k];
}

/*******************************************************************************
* Function Name  : get3DMatrix
* Description    : get 3DMatrix
* Input          : None 
* Output         : vector< vector <vector<int> > > 
* Return			   : _3Dmatrix, 3D Led Matrix
*******************************************************************************/
char ***C3DLedMatrix::get3DMatrix(void)
{
	return _3Dmatrix;
}
