/*

How use this module:
	1 -> set3DMatrix(char ***); set the 3D Matrix that will be outputed
	2 -> write3DMatrix(); write in physical 3D Matrix
*/

#ifndef _C3DLEDMATRIX_H
#define _C3DLEDMATRIX_H

#define __ROWS		5
#define __COLUMNS	5
#define __LAYERS 	5

#include "C2DLedMatrix.h"
#include "CLayer.h"

class C3DLedMatrix : public CLayer, public C2DLedMatrix 
{
public:

	C3DLedMatrix();
	~C3DLedMatrix();

	void write3DMatrix(void);
	void set3DMatrix( char ***);
	char ***get3DMatrix(void);

private:
	char ***_3Dmatrix;
};

#endif //_C3DLEDMATRIX_H

