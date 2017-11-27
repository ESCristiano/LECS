#include "CInitializations.h"

CInitializations::CInitializations()
{
	
}


CInitializations::~CInitializations()
{
	
}

/*******************************************************************************
* Function Name  : init3DLedMatrix
* Description    : Initialize 3D Led Matrix
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CInitializations::init3DLedMatrix()
{
	
}

/*******************************************************************************
* Function Name  : initCapacitiveSensor
* Description    : Initialize Capacitive Sensor
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CInitializations::initCapacitiveSensor()
{
	
}

/*******************************************************************************
* Function Name  : initMicrophone
* Description    : Initialize Microphone
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CInitializations::initMicrophone()
{
	
}

/*******************************************************************************
* Function Name  : initLightSensor
* Description    : Initialize Light Sensor
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CInitializations::initLightSensor()
{
	CLdrSensor ldr;
	
	ldr.initLDR();
	ldr.initTimer();
}

void vLEDTask( void *pvParameters );

/*******************************************************************************
* Function Name  : initTasks()
* Description    : Create Tasks and Assign Tasks Priorities
* Input          : None (void)
* Output         : int
* Return				 : Return 0 if everything went well
*								 : Return -1 if occurrer one error
*******************************************************************************/
int CInitializations::initTasks()
{
	portBASE_TYPE task1_pass;
	
	/* Create Task */
	task1_pass = xTaskCreate( vLEDTask, "Task_Led", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	if( task1_pass == pdPASS )
	{
			/* Everything went well*/
			return 0;
	}
	else
	{
			/* ERROR! Creating the Tasks */
			return -2;
	}
	
}


void vLEDTask( void *pvParameters )
{

	CLeds leds;
	
	for( ;; )
	{
		if(value >200)
		{
			leds.resetBlue();
			leds.resetGreen();
			leds.resetRed();
			leds.setOrange();
		}			
		else
			if(value >150)
			{
				leds.resetBlue();
				leds.resetGreen();
				leds.resetOrange();
				leds.setRed();
			
			}				
			else
				if(value >100)
				{
					leds.resetRed();
					leds.resetGreen();
					leds.resetOrange();
					leds.setBlue();	
				}
				else
					if(value >= 0)
					{	
					leds.resetRed();
					leds.resetBlue();
					leds.resetOrange();
					leds.setGreen();	
					}
	}
}

