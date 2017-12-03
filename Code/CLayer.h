#ifndef _CLAYER_H
#define _CLAYER_H

class CLayer {
public:
	CLayer();
	~CLayer();
	void setLayer(int*);
	int* getLayer(void);

protected:
	int layer[5];
	void writeLayer(int);
};

#endif //_CLAYER_H
