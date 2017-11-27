#ifndef __CLDR_SENSOR_H__
#define __CLDR_SENSOR_H__

/*
http://www.micromouseonline.com/2016/02/03/tim3-arr-regular-interrupts-stm32f4/

Ldr is read at frequency of 1Hz

How use this module:
	1 -> initLDR(); Configure ADC and Input Pin
	2 -> initTimer(); Configure Timer Interrupt
	3 -> readLDR();
	4 -> closeLDR();
*/

/*-------------Includes ---------------*/
#include <stm32f4xx_adc.h>

extern uint16_t value;

class CLdrSensor
{
	public:
		CLdrSensor();
		~CLdrSensor();
		void initLDR(void);
		void initTimer(void);
		uint16_t readLDR(void);	
		void closeLDR(void);
	private:
		uint16_t mvalueLdr;
		void timerInterruptInit(void);
		void timerInit(void);
		void timerStart(void);
		void timerStop(void);
		void timerInterruptEnable(void);
		void timerInterruptDisable(void);	
};

#endif // __CLDR_SENSOR_H__

/*++++++++++++++++++++++++Functions Description+++++++++++++++++++++++++++++++++*/

/*******************************************************************************
	* Function Name  : init_LDR
	* Description    : Initialization of the adc 1 channel 1 PA1 for read LDR
	* Input          : None (void)
	* Output         : None (void)
	* Return				 : None
	*******************************************************************************/
	//void init_LDR(void);

/*******************************************************************************
* Function Name  : initTimer
* Description    : Initialization timer 7 and her interrupt
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::initTimer(void);

	/*******************************************************************************
	* Function Name  : read_LDR
	* Description    : Initialization of the adc for read LDR
	* Input          : None (void)
	* Output         : uint16_t
	* Return				 : Value between 0 and 255 that represent the value of LDR
	*******************************************************************************/
	//uint16_t read_LDR(void);

	/*******************************************************************************
	* Function Name  : close_LDR 
	* Description    : Cleanup
	* Input          : None (void)
	* Output         : None (void)
	* Return				 : None
	*******************************************************************************/
	//void close_LDR(void);
	
	/*******************************************************************************
* Function Name  : timerInterruptInit
* Description    : Init timer 7 overflow interrupt
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::timerInterruptInit(void);

/*******************************************************************************
* Function Name  : timerInit
* Description    : Init timer 7
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::timerInit(void);

/*******************************************************************************
* Function Name  : timerStart
* Description    : Run timer 7
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::timerStart(void);

/*******************************************************************************
* Function Name  : timerStop
* Description    : Stop Timer 7
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::timerStop(void);

/*******************************************************************************
* Function Name  : timerInterruptEnable
* Description    : Enable timer 7 interreput
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::timerInterruptEnable(void);

/*******************************************************************************
* Function Name  : timerInterruptDisable
* Description    : Disable timer 7 interreput
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::timerInterruptDisable(void);

/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : ISR
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
//void CLdrSensor::TIM7_IRQHandler(void);

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
