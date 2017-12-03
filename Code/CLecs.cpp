#include "CLecs.h"
#include "CSensors.h"
#include "CBoardLeds.h"
#include "CLightSensor.h"

CLecs::CLecs()
{
}


CLecs::~CLecs()
{
}

/*******************************************************************************
* Function Name  : initNVIC
* Description    : configure a priority group. This function should be called
*								 :before any other and before RTOS start
* Input          : None (void)
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CLecs::initNVIC()
{
/*******https://www.freertos.org/RTOS-Cortex-M3-M4.html**********
	It is recommended to assign all the priority bits to be preempt priority bits, leaving no priority bits as subpriority bits. Any other configuration complicates the 
	otherwise direct relationship between the configMAX_SYSCALL_INTERRUPT_PRIORITY setting and the priority assigned to individual peripheral interrupts.
	Most systems default to the wanted configuration, with the noticeable exception of the STM32 driver library. If you are using an STM32 with the STM32 driver library 
	then ensure all the priority bits are assigned to be preempt priority bits by calling NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
	
	information given by entity that provide freertos
	*/
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
}

/*******************************************************************************
* Function Name  : init3DLedMatrix
* Description    : Initialize 3D Led Matrix
* Input          : None (void)
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CLecs::init3DLedMatrix()
{
}

/*******************************************************************************
* Function Name  : initLecsSensors
* Description    : Initialize all sensors
* Input          : None (void)
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CLecs::initLecsSensors()
{
	CSensors* sensors = CSensors::getInstance();
	sensors->initSensors();
}

/*******************************************************************************
* Function Name  : initSemaphores
* Description    : Initialize all semaphores
* Input          : None (void)
* Output         : None (void)
* Return		 : None
*******************************************************************************/
void CLecs::initSemaphores()
{
	extern SemaphoreHandle_t Sem_ISR_3D;  				//When an interrupt occur this semaphore is released and the Update Matrix Task, leave your state of wait, and execute your function.
	extern SemaphoreHandle_t Sem_ISR_Sleep;   		//when a wake up condition is verify this semaphore is released and the programme wake up and start running normally.
	extern SemaphoreHandle_t Sem_DataMining_Sleep;// When a sleep condition is verify this semaphore is released and the programme go to a sleep mode.

	/*Binary Semaphore Creations*/
	Sem_ISR_3D = xSemaphoreCreateBinary();
	Sem_ISR_Sleep = xSemaphoreCreateBinary();
	Sem_DataMining_Sleep = xSemaphoreCreateBinary();
}

/*******************************************************************************
* Function Name  : initQueue
* Description    : Initialize all queue
* Input          : None (void)
* Output         : None (void)
* Return			   : None
*******************************************************************************/
void CLecs::initQueue()
{
	extern xQueueHandle Queue_ISR_CapSensor;						//Send the read data from capacitive sensors to Cap Sensor Task.
	extern xQueueHandle Queue_ISR_LDR;      					 	//Send the read data from LDR sensor to LDR Task.
	extern xQueueHandle Queue_DMA_ProcessData;   			 	//Send the read data from Microphone sensor to Process Data Task.
	extern xQueueHandle Queue_SensorFusion_DataMining;	//Send the read data from capacitive sensors, LDR sensor and microphone sensor in one only structure of data that was fused.
	extern xQueueHandle Queue_DataMining_Make;					//Send the pattern that we obtained with analysis the data to Make Graph Task.
	
	/*-------------------------------------------------------------------------------------
	QueueHandle_t xQueueCreate( UBaseType_t uxQueueLength,
                             UBaseType_t uxItemSize );
	
			uxQueueLength  -> The maximum number of items the queue can hold at any one time.
			uxItemSize  	 -> The size, in bytes, required to hold each item in the queue.
	---------------------------------------------------------------------------------------*/
	
	/*Queue creation with respective data types*/
	Queue_ISR_CapSensor = xQueueCreate(4	, 4*sizeof( char ) );
	Queue_ISR_LDR = xQueueCreate(4	,sizeof( uint16_t ) );
	//Queue_DMA_ProcessData = xQueueCreate(4	,sizeof(  ) );  ainda não sei o tamanho de cada elemento da message
	//Queue_SensorFusion_DataMining = xQueueCreate(4	,sizeof(  ) );  ainda não sei o tamanho de cada elemento da message
	//Queue_DataMining_Make = xQueueCreate(4	,sizeof(  ) );	  ainda não sei o tamanho de cada elemento da message
}

void vLEDTask( void *pvParameters )
{
	char brightness;
	extern xQueueHandle Queue_ISR_LDR;
	uint16_t valueLdr;
	SemaphoreHandle_t mutexSensors;
	mutexSensors = xSemaphoreCreateMutex();
	CLeds leds;
	
	for( ;; )
	{
		
		if(uxQueueMessagesWaiting(Queue_ISR_LDR))
		{ 
		//	leds.toggleBlue();
			xQueueReceive( Queue_ISR_LDR, &valueLdr, 0 );
			/*
				The brightness scale is split in 8.
															ADC value (LDR) 
				brightness level 0 	-> 	[0 to 32]
				brightness level 1 	-> 	[33 to 64] 
				brightness level 2 	-> 	[65 to 96]
				brightness level 3 	-> 	[97 to 128]
				brightness level 4 	-> 	[129 to 160]
				brightness level 5 	-> 	[161 to 192]
				brightness level 6 	-> 	[193 to 224]
				brightness level 7 	-> 	[225 to 255]
			
				The mask (& 0x7) certifie that the brightness range is between 0 and 7
			*/
			brightness = (valueLdr/33) & 0x7;
			
			static CSensors* Sensors = CSensors::getInstance(); //static because the pointer just can be initialized once
		
			/*Lock Mutex*/
			if( xSemaphoreTake( mutexSensors, ( TickType_t ) 10 ) == pdTRUE )
        {
          Sensors->setDataLdr(brightness);  
					/*Unlock Mutex*/
					xSemaphoreGive( mutexSensors );
        }
		}
	}
}

/*******************************************************************************
* Function Name  : vLDRTask
* Description    : call function of the task LDRTask
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vLDRTask(void *pvParameters)
{
	CLeds leds;
	CSensors* Sensors = CSensors::getInstance();
	for( ;; )
	{
//		leds.setRed();
//		if(Sensors->getDataLdr() == 3)
//			{
//				leds.resetBlue();
//				leds.resetGreen();
//				leds.resetRed();
//				leds.setOrange();
//			}			
//			else
//				if(Sensors->getDataLdr()  == 2)
//				{
//					leds.resetBlue();
//					leds.resetGreen();
//					leds.resetOrange();
//					leds.setRed();
//				
//				}				
//				else
//					if(Sensors->getDataLdr()  == 1)
//					{
//						leds.resetRed();
//						leds.resetGreen();
//						leds.resetOrange();
//						leds.setBlue();	
//					}
//					else
//						{	
//						leds.resetRed();
//						leds.resetBlue();
//						leds.resetOrange();
//						leds.setGreen();	
//						}
		}
}

 //só para fins de testes
void vLEDTask2( void *pvParameters )
{
	
	CLeds leds;
	leds.setGreen();
	leds.setBlue();
	for( ;; )
	{
		leds.setGreen();
	}
}
/*******************************************************************************
* Function Name  : initTasks()
* Description    : Create Tasks and Assign Tasks Priorities
* Input          : None (void)
* Output         : int
* Return		     : Return 0 if everything went well
*				         : Return -2 if occurrer one error
*******************************************************************************/
int CLecs::initTasks()
{
	portBASE_TYPE task1_pass, task_test;
	
	/* Create Task */
	task1_pass = xTaskCreate(vLDRTask, "LDRTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//  task_test = xTaskCreate(vTaskTest, "Task_test", configMINIMAL_STACK_SIZE, NULL, 1, NULL); //só para fins de testes
//	xTaskCreate(vLEDTask2, "Task_Led2", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	
	if ((task_test == pdPASS) && (task1_pass == pdPASS))
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

/*******************************************************************************
* Function Name  : run
* Description    : Start schedule
* Input          : None (void)
* Output         : None (void)
* Return		     : None
*******************************************************************************/
int CLecs::run()
{
	if (!this->initTasks())
	{
		/* Start the Scheduler */
		vTaskStartScheduler();
		return 0;
	}
	else
	{
		/* ERROR! Creating the Tasks */
		return -2;
	}
}

CLecs* CLecs::instance = 0;

CLecs * CLecs::getInstance()
{
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton
	if (instance == 0)
		instance = new CLecs;
	//mutex unclock 
	return instance;
}

