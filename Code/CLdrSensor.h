#ifndef __CLDR_SENSOR_H__
#define __CLDR_SENSOR_H__

/*-------------Includes ---------------*/
#include <stm32f4xx_adc.h>

class CLdrSensor
{
public:
	CLdrSensor();
	~CLdrSensor();

	/*******************************************************************************
	* Function Name  : init_LDR
	* Description    : Initialization of the adc 1 channel 1 PA1 for read LDR
	* Input          : None (void)
	* Output         : None (void)
	* Return				 : None
	*******************************************************************************/
	void init_LDR(void);

	/*******************************************************************************
	* Function Name  : read_LDR
	* Description    : Initialization of the adc for read LDR
	* Input          : None (void)
	* Output         : uint16_t
	* Return				 : Value between 0 and 255 that represent the value of LDR
	*******************************************************************************/
	int read_LDR(void);

	/*******************************************************************************
	* Function Name  : close_LDR 
	* Description    : Cleanup
	* Input          : None (void)
	* Output         : None (void)
	* Return				 : None
	*******************************************************************************/
	void close_LDR(void);
};

#endif // __CLDR_SENSOR_H__
