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

/*******************************************************************************
* Function Name  : setLayer
* Description    : write in private menber layer[]
* Input          : int index, indicate the layer to be set 
* Output         : None 
* Return			   : None
*******************************************************************************/
void CLayer::setLayer(int index)
{
	for(int i = 0; i < 5; i++)
		layer[i] = 1;
	layer[index] = 0; //lógica negativa porque usamos Mosfets P-Channel
}

/*******************************************************************************
* Function Name  : writeLayer
* Description    : set one layer
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void CLayer::writeLayer()
{
	/*Layers
	
		layer 0 PE6	 output		11	GPIO		LS_0
		layer 1 PC13 output	  12	GPIO		LS_1
		layer 2 PE4	 output		13	GPIO		LS_2
		layer 3 PE5	 output		14	GPIO		LS_3
		layer 4 PE3	 output		15	GPIO		LS_4
	
	*/
	
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction) layer[0]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_3,(BitAction) layer[1]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_6, (BitAction) layer[2]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_4, (BitAction) layer[3]);
	GPIO_WriteBit(GPIOE, GPIO_Pin_5, (BitAction) layer[4]);


}
