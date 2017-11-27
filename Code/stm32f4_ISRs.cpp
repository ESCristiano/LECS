#include "stm32f4_ISRs.h"

/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : ISR
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
// extern "C" -> for C++, ensure the interrupt handler is linked as a C function
extern "C" void TIM7_IRQHandler(void) 
{
	CLdrSensor ldr;
	if (TIM_GetITStatus (TIM7, TIM_IT_Update) != RESET) {
    value = ldr.readLDR();
		//message queue
    TIM_ClearITPendingBit (TIM7, TIM_IT_Update);
  }
}
