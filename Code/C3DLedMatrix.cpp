#include "C3DLedMatrix.h"



C3DLedMatrix::C3DLedMatrix()
{

	/*Create a empty vector
			{ 0, 0, 0, 0, 0 }
	*/
	vector<int> row(5,0); 
	vector< vector<int> > matrix;
	
		/*  Initialize 2D Led Matrix
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 } 
	*/
	for (int i = 0; i < 5; i++)
	{
		matrix.push_back(row);
	}
	
		/*  Initialize 3D Led Matrix
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }
			{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 }	{ 0, 0, 0, 0, 0 } 
	*/
	for (int i = 0; i < 5; i++)
	{
		_3Dmatrix.push_back(matrix);
	}
}


C3DLedMatrix::~C3DLedMatrix()
{
}

/*******************************************************************************
* Function Name  : write3DMatrix
* Description    : set one 3DMatrix
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void C3DLedMatrix::write3DMatrix(void)
{
	for (int i = 0; i < 5; i++)
	{
		this->setLayer(i);
		this->set2DMatrix(_3Dmatrix[i]);
		this->writeLayer();
		this->write3DMatrix();
	}
	
}

/*******************************************************************************
* Function Name  : setLayer
* Description    : write in private menber _2Dmatrix
* Input          : vector< vector< vector<int> > > matrix3d, indicate the 3D 
*								 : matrix to be set 
* Output         : None 
* Return			   : None
*******************************************************************************/
void C3DLedMatrix::set3DMatrix(vector< vector< vector<int> > > matrix3d)
{
	_3Dmatrix = matrix3d;
}

/*******************************************************************************
* Function Name  : get3DMatrix
* Description    : get 3DMatrix
* Input          : None 
* Output         : vector< vector <vector<int> > > 
* Return			   : _3Dmatrix, 3D Led Matrix
*******************************************************************************/
vector< vector <vector<int> > > C3DLedMatrix::get3DMatrix(void)
{
	return _3Dmatrix;
}
