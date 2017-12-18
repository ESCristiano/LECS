/*

How use this module:
	1 -> set2DMatrix(char **); set the 2D Matrix that will be outputed
	2 -> write2DMatrix();  write in physical 2D Matrix
*/

#ifndef _C2DLEDMATRIX_H
#define _C2DLEDMATRIX_H

#define __ROWS		5
#define __COLUMNS	5

class C2DLedMatrix
{
public:
	C2DLedMatrix();
	~C2DLedMatrix();

protected:
	void set2DMatrix(char **);
	void write2DMatrix();

private:
	char **_2Dmatrix; 
};

#endif //_C2DLEDMATRIX_H
