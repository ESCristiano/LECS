#include "CLayer.h"

/* Library includes. */
#include <stm32f4xx.h>

CLayer::CLayer()
{
	for(int i = 0; i < 5; i++)
		layer[i] = 1;
}

CLayer::~CLayer()
{
}

void CLayer::writeLayer(int index)
{
	/*Layers
	
		layer 0 led 0	PE6	 output		11	GPIO		LS_0
		layer 1 led 1	PC13 output	  12	GPIO		LS_1
		layer 2 led 2	PE4	 output		13	GPIO		LS_2
		layer 3 led 3	PE5	 output		14	GPIO		LS_3
		layer 4 led 4	PE2	 output		15	GPIO		LS_4
	
	*/
	
	for(int i = 0; i < 5; i++)
		layer[i] = 1;
	layer[index] = 0; //lógica negativa porque usamos Mosfets P-Channel
	
	GPIO_WriteBit(GPIOE, GPIO_Pin_6, (BitAction) layer[0]);
	GPIO_WriteBit(GPIOC, GPIO_Pin_13,(BitAction) layer[1]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_4, (BitAction) layer[2]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_5, (BitAction) layer[3]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction) layer[4]);


}
