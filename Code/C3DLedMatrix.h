#ifndef _C3DLEDMATRIX_H
#define _C3DLEDMATRIX_H

#include "CLayer.h"
#include "C2DLedMatrix.h"

#include <vector>
using namespace std;

class C3DLedMatrix : public CLayer, public C2DLedMatrix 
{
public:

	C3DLedMatrix();
	~C3DLedMatrix();

	void write3DMatrix(void);
	void set3DMatrix( vector< vector <vector<int> > > );
	vector< vector <vector<int> > > get3DMatrix(void);

private:
	vector< vector< vector<int> > > _3Dmatrix;
};

#endif //_C3DLEDMATRIX_H

