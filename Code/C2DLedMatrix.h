
#ifndef _C2DLEDMATRIX_H
#define _C2DLEDMATRIX_H

#include <vector>
using namespace std;

class C2DLedMatrix
{
public:
	C2DLedMatrix();
	~C2DLedMatrix();

//protected:
	void set2DMatrix(vector< vector<int> >);
	void write2DMatrix();

private:
	vector< vector<int> > _2Dmatrix;

};

#endif //_C2DLEDMATRIX_H
