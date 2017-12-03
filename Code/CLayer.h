/*

*/
#ifndef _CLAYER_H
#define _CLAYER_H

class CLayer {
	
public:
	CLayer();
	~CLayer();
protected:
	void writeLayer();
	void setLayer(int);
private:
	int layer[5];
};

#endif //_CLAYER_H
