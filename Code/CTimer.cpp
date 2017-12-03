#include "CTimer.h"

CTimer::CTimer()
{
}


CTimer::~CTimer()
{
}

/*******************************************************************************
* Function Name  : timerInterruptInit
* Description    : Init timer 7 overflow interrupt
* Input          : uint8_t HANDLERx name of the interrupt, see notes.txt
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CTimer::timerInterruptInit(uint8_t HANDLERx)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the timer global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = HANDLERx;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15; //lowest priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;//lowest priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : timerInit
* Description    : Init timer 7
* Input          : uint16_t prescaler 
*				 : uint16_t reload -> reload time to generate a overflow interrupt
*				 : uint32_t RCC_APB1Periph_x -> clock do timer
*				 : TIM_TypeDef* TIMx -> indicate the timer that we want use
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CTimer::timerInit(uint16_t prescaler, uint16_t reload, uint32_t RCC_APB1Periph_x, TIM_TypeDef* TIMx)
{
	/*
	TIM7 Clock Frequency = 84MHZ
	Prescaler = 42000
	(TIM7 Clock Frequency) / Prescaler = 2000 HZ
	New Frequency = 2000 HZ => ( 0.5 ms for tick )
	To have 1 second = 0.5 ms * 2000
	So, put 2000 in reload.
	*/

	//uint16_t prescaler = 42000 - 1;
	//uint16_t reload = (2000) - 1; //overflow 1 second in 1 second

								  /*Enable the clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_x, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* set everything back to default values */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	/* only changes from the defaults are needed */
	TIM_TimeBaseStructure.TIM_Period = reload - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
}

/*******************************************************************************
* Function Name  : timerStart
* Description    : Run timer 7
* Input          : TIM_TypeDef* TIMx -> indicate the timer that we want use
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CTimer::timerStart(TIM_TypeDef* TIMx)
{
	TIM_Cmd(TIMx, ENABLE);
}

/*******************************************************************************
* Function Name  : timerStop
* Description    : Stop Timer 7
* Input          : TIM_TypeDef* TIMx -> indicate the timer that we want use
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CTimer::timerStop(TIM_TypeDef* TIMx)
{
	TIM_Cmd(TIMx, DISABLE);
}

/*******************************************************************************
* Function Name  : timerInterruptEnable
* Description    : Enable timer 7 interreput
* Input          : TIM_TypeDef* TIMx -> indicate the timer that we want use
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CTimer::timerInterruptEnable(TIM_TypeDef* TIMx)
{
	/*
	* It is important to clear any pending interrupt flags since the timer
	* has been free-running since we last used it and that will generate
	* interrupts on overflow even though the associated interrupt event has
	* not been enabled.
	*/
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);
	/* put the counter into a known state */
	TIM_SetCounter(TIMx, 0);
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
}

/*******************************************************************************
* Function Name  : timerInterruptDisable
* Description    : Disable timer 7 interreput
* Input          : TIM_TypeDef* TIMx -> indicate the timer that we want use
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CTimer::timerInterruptDisable(TIM_TypeDef* TIMx)
{
	TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
}
