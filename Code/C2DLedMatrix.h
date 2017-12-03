
#ifndef _C2DLEDMATRIX_H
#define _C2DLEDMATRIX_H

#include <vector>
using namespace std;

class C2DLedMatrix
{
public:
	C2DLedMatrix();
	~C2DLedMatrix();
	void set2DMatrix(vector< vector<int> >);
	vector< vector<int> > get2DMatrix(void);

protected:
	vector< vector<int> > _2Dmatrix;
	void write2DMatrix(vector< vector<int> >);

};

#endif //_C2DLEDMATRIX_H
