#include "C2DLedMatrix.h"

/* Library includes. */
#include <stm32f4xx.h>

C2DLedMatrix::C2DLedMatrix()
{
	
	/*  Allocate memory and 
	 Initialize 2D Led Matrix
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 } 
	*/
	
	_2Dmatrix = new char*[5];

	for (int i = 0; i < 5; ++i)
		_2Dmatrix[i] = new char[5](); // allocate and set all elements 0
}


C2DLedMatrix::~C2DLedMatrix()
{
	/*deallocate matrix*/
	for (int i = 0; i < 5; ++i)
    delete [] _2Dmatrix[i];
	delete [] _2Dmatrix;
}

/*******************************************************************************
* Function Name  : setLayer
* Description    : write in private menber _2Dmatrix
* Input          : vector< vector<int> > matrix, indicate the matrix to be set 
* Output         : None 
* Return			   : None
*******************************************************************************/
void C2DLedMatrix::set2DMatrix(char **matrix)
{
	/*
		copy matrix to _2Dmatrix
		_2Dmatrix = matrix
	*/
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
			_2Dmatrix[i][j] = matrix[i][j];
	
	}
}

/*******************************************************************************
* Function Name  : write2DMatrix
* Description    : write in physical 2D Matrix
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void C2DLedMatrix::write2DMatrix()
{
	/*Columns
	
	column led 0	PA3		output	13		GPIO		CS_0
	column led 1	PA2		output	14		GPIO		
	column led 2	PC5		output	19		GPIO		
	column led 3	PC4		output	20		GPIO		
	column led 4	PB1		output	21		GPIO		
								
	column led 5	PB0		output	22		GPIO		
	column led 6	PE7		output	25		GPIO		
	column led 7	PE8		output	26		GPIO		
	column led 8	PE9		output	27		GPIO		
	column led 9	PE10	output	28		GPIO		
								
	column led 10	PE11	output	29		GPIO		
	column led 11	PE12	output	30		GPIO		
	column led 12	PE13	output	31		GPIO		
	column led 13	PE14	output	32		GPIO		
	column led 14	PE15	output	33		GPIO		
								
	column led 15	PB11	output	35		GPIO		
	column led 16	PB12	output	36		GPIO		
	column led 17	PB13	output	37		GPIO		
	column led 18	PB14	output	38		GPIO		
	column led 19	PB15	output	39		GPIO		
								
	column led 20	PD8		output	40		GPIO		
	column led 21	PD9		output	41		GPIO		
	column led 22	PD10	output	42		GPIO		
	column led 23	PD11	output	43		GPIO		
	column led 24	PD12	output	44		green led	*	CS_24
	
	*/

	GPIO_WriteBit(GPIOA, GPIO_Pin_2, 	(BitAction) _2Dmatrix[0][0]);
	GPIO_WriteBit(GPIOA, GPIO_Pin_3, 	(BitAction) _2Dmatrix[0][1]);
	GPIO_WriteBit(GPIOC, GPIO_Pin_4, 	(BitAction) _2Dmatrix[0][2]);
	GPIO_WriteBit(GPIOC, GPIO_Pin_5, 	(BitAction) _2Dmatrix[0][3]);
	GPIO_WriteBit(GPIOB, GPIO_Pin_0, 	(BitAction) _2Dmatrix[0][4]);
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_1, 	(BitAction) _2Dmatrix[1][0]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_8,	(BitAction) _2Dmatrix[1][1]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_7, 	(BitAction) _2Dmatrix[1][2]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_10, (BitAction) _2Dmatrix[1][3]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_9, 	(BitAction) _2Dmatrix[1][4]);
	
	GPIO_WriteBit(GPIOE, GPIO_Pin_12, (BitAction) _2Dmatrix[2][0]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_11,	(BitAction) _2Dmatrix[2][1]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_14, (BitAction) _2Dmatrix[2][2]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_13, (BitAction) _2Dmatrix[2][3]);
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, 	(BitAction) _2Dmatrix[2][4]);
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction) _2Dmatrix[3][0]);
	GPIO_WriteBit(GPIOB, GPIO_Pin_11,	(BitAction) _2Dmatrix[3][1]);
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction) _2Dmatrix[3][2]);
	GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction) _2Dmatrix[3][3]);
	GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction) _2Dmatrix[3][4]);
	
	GPIO_WriteBit(GPIOD, GPIO_Pin_8, 	(BitAction) _2Dmatrix[4][0]);
	GPIO_WriteBit(GPIOD, GPIO_Pin_9,	(BitAction) _2Dmatrix[4][1]);
	GPIO_WriteBit(GPIOD, GPIO_Pin_10, (BitAction) _2Dmatrix[4][2]);
	GPIO_WriteBit(GPIOD, GPIO_Pin_11, (BitAction) _2Dmatrix[4][3]);
	GPIO_WriteBit(GPIOD, GPIO_Pin_12, (BitAction) _2Dmatrix[4][4]);
	
//	GPIO_WriteBit(GPIOA, GPIO_Pin_3, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOA, GPIO_Pin_2, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOC, GPIO_Pin_5, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOC, GPIO_Pin_4, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOB, GPIO_Pin_1, 	(BitAction) 0);
//	
//	GPIO_WriteBit(GPIOB, GPIO_Pin_0, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_7,	(BitAction) 0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_8, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_9, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_10, (BitAction) 0);
//	
//	GPIO_WriteBit(GPIOE, GPIO_Pin_11, (BitAction) 0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_12,	(BitAction) 0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_13, (BitAction)0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_14, (BitAction) 0);
//	GPIO_WriteBit(GPIOE, GPIO_Pin_15, (BitAction) 0);
//	
//	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction) 0);
//	GPIO_WriteBit(GPIOB, GPIO_Pin_12,	(BitAction) 0);
//	GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction) 0);
//	GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction) 0);
//	GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction) 0);
//	
//	GPIO_WriteBit(GPIOD, GPIO_Pin_8, 	(BitAction) 0);
//	GPIO_WriteBit(GPIOD, GPIO_Pin_9,	(BitAction) 0);
//	GPIO_WriteBit(GPIOD, GPIO_Pin_10, (BitAction) 0);
//	GPIO_WriteBit(GPIOD, GPIO_Pin_11, (BitAction)0);
//	GPIO_WriteBit(GPIOD, GPIO_Pin_12, (BitAction) 0);
	
}

