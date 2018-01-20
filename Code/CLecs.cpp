#include <stdlib.h>

#include "CLecs.h"
#include "CTimer.h"
#include "CSensors.h"
#include "CBoardLeds.h"
#include "CMicrophone.h"
#include "CLightSensor.h"
#include "C3DLedMatrix.h"
#include "C3DLedMatrixBuffer.h"

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
* Return		 	   : None
*******************************************************************************/
void CLecs::init3DLedMatrix()
{
	// Enabling GPIO peripherals clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //Columns
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //Columns
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //Layer and columns
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //Columns
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //Layer and columns
	
	/*Init Layers*******************************************************************
	
		layer 0 PE6	 output		11	GPIO		LS_0
		layer 1 PC13 output	  12	GPIO		LS_1
		layer 2 PE4	 output		13	GPIO		LS_2
		layer 3 PE5	 output		14	GPIO		LS_3
		layer 4 PE3	 output		15	GPIO		LS_4
	
	*/
	
	/*Layers in GPIOC*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct;
	// GPIO peripheral properties specification
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // pullup/pulldown resistors inactive
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/*Layers in GPIOE*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct_1;
	// GPIO peripheral properties specification
	GPIO_InitStruct_1.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6; 
	GPIO_InitStruct_1.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct_1.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct_1.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct_1.GPIO_PuPd = GPIO_PuPd_UP; // pullup/pulldown resistors inactive
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOE, &GPIO_InitStruct_1);
	
	/*******************************************************************************/
	
	/*Init Columns******************************************************************
	
	column led 0	PA3		output	13		GPIO		CS_0
	column led 1	PA2		output	14		GPIO		
	column led 2	PC5		output	19		GPIO		
	column led 3	PC4		output	20		GPIO		
	column led 4	PB1		output	21		GPIO		
								
	column led 5	PB0		output	22		GPIO		
	column led 6	PE7		output	25		GPIO		
	column led 7	PE8		output	26		GPIO		
	column led 8	PE9		output	27		GPIO		
	column led 9	PE10	output	28		GPIO		
								
	column led 10	PE11	output	29		GPIO		
	column led 11	PE12	output	30		GPIO		
	column led 12	PE13	output	31		GPIO		
	column led 13	PE14	output	32		GPIO		
	column led 14	PE15	output	33		GPIO		
								
	column led 15	PB11	output	35		GPIO		
	column led 16	PB12	output	36		GPIO		
	column led 17	PB13	output	37		GPIO		
	column led 18	PB14	output	38		GPIO		
	column led 19	PB15	output	39		GPIO		
								
	column led 20	PD8		output	40		GPIO		
	column led 21	PD9		output	41		GPIO		
	column led 22	PD10	output	42		GPIO		
	column led 23	PD11	output	43		GPIO		
	column led 24	PD12	output	44		green led	*	CS_24
	
	*/
	/*Layers in GPIOA*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct_2;
	// GPIO peripheral properties specification
	GPIO_InitStruct_2.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct_2.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct_2.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct_2.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct_2.GPIO_PuPd = GPIO_PuPd_DOWN; // pulldown resistors 
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOA, &GPIO_InitStruct_2);
	
	/*Layers in GPIOB*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct_3;
	// GPIO peripheral properties specification
	GPIO_InitStruct_3.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	GPIO_InitStruct_3.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct_3.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct_3.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct_3.GPIO_PuPd = GPIO_PuPd_DOWN; // pulldown resistors 
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOB, &GPIO_InitStruct_3);
	
	/*Layers in GPIOC*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct_4;
	// GPIO peripheral properties specification
	GPIO_InitStruct_4.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct_4.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct_4.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct_4.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct_4.GPIO_PuPd = GPIO_PuPd_DOWN; // pulldown resistors 
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOC, &GPIO_InitStruct_4);
	
	/*Layers in GPIOD*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct_5;
	// GPIO peripheral properties specification
	GPIO_InitStruct_5.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; 
	GPIO_InitStruct_5.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct_5.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct_5.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct_5.GPIO_PuPd = GPIO_PuPd_DOWN; // pulldown resistors 
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOD, &GPIO_InitStruct_5);
	
	/*Layers in GPIOE*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct_6;
	// GPIO peripheral properties specification
	GPIO_InitStruct_6.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStruct_6.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct_6.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct_6.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct_6.GPIO_PuPd = GPIO_PuPd_DOWN; // pulldown resistors 
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOE, &GPIO_InitStruct_6);
	/*******************************************************************************/
	
	CTimer timer6; //basic timer (timer 6 or timer 7)
	
	/*Interrupt in 1ms in 1ms*/
	timer6.timerInit(42000, 2, RCC_APB1Periph_TIM6, TIM6);
	timer6.timerInterruptInit(TIM6_DAC_IRQn);
	timer6.timerInterruptEnable(TIM6);
	timer6.timerStart(TIM6);
	
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
	extern SemaphoreHandle_t Sem_ISR_ChangePattern; //When an interrupt occur this semaphore is released and the change pattern, leave your state of wait, and execute your function.
	extern SemaphoreHandle_t Sem_ISR_Sleep;   		//when a wake up condition is verify this semaphore is released and the programme wake up and start running normally.
	extern SemaphoreHandle_t Sem_ISR_ProcessData; //Send the read data from Microphone sensor to Process Data Task.
	extern SemaphoreHandle_t Sem_DataMining_Sleep;// When a sleep condition is verify this semaphore is released and the programme go to a sleep mode.

	/*Binary Semaphore Creations*/
	Sem_ISR_3D = xSemaphoreCreateBinary();
	Sem_ISR_ChangePattern = xSemaphoreCreateBinary();
	Sem_ISR_Sleep = xSemaphoreCreateBinary();
	Sem_ISR_ProcessData = xSemaphoreCreateBinary();
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
	extern xQueueHandle Queue_SensorFusion_DataMining;	//Send the read data from capacitive sensors, LDR sensor and microphone sensor in one only structure of data that was fused.
	extern xQueueHandle Queue_DataMining_Make;					//Send the pattern that we obtained with analysis the data to Make Graph Task.
	
	/*-------------------------------------------------------------------------------------
	QueueHandle_t xQueueCreate( UBaseType_t uxQueueLength,
                             UBaseType_t uxItemSize );
	
			uxQueueLength  -> The maximum number of items the queue can hold at any one time.
			uxItemSize  	 -> The size, in bytes, required to hold each item in the queue.
	---------------------------------------------------------------------------------------*/
	
	/*Queue creation with respective data types*/
	Queue_ISR_CapSensor = xQueueCreate(4	, 1*sizeof( uint8_t ) );
	Queue_ISR_LDR = xQueueCreate(4	,sizeof( uint16_t ) );
	Queue_SensorFusion_DataMining = xQueueCreate(4	,sizeof( char) );  
	Queue_DataMining_Make = xQueueCreate(4	,sizeof(char) );	 
}

/*******************************************************************************
* Function Name  : initMutexs
* Description    : Initialize all mutexs
* Input          : None (void)
* Output         : None (void)
* Return			   : None
*******************************************************************************/
void CLecs::initMutexs()
{
	extern SemaphoreHandle_t mutex3DPattern;
	extern SemaphoreHandle_t mutexSensors; 
		
	mutex3DPattern =  xSemaphoreCreateMutex();
	mutexSensors = xSemaphoreCreateMutex();
}

/*******************************************************************************
* Function Name  : vLDRTask
* Description    : call function of the task LDRTask
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vLDRTask( void *pvParameters )
{
	char brightness;
	extern xQueueHandle Queue_ISR_LDR;
	extern SemaphoreHandle_t mutexSensors; 
	uint16_t valueLdr;
	CLeds led;
	
	for( ;; )
	{		
		if(uxQueueMessagesWaiting(Queue_ISR_LDR))
		{ 
			xQueueReceive( Queue_ISR_LDR, &valueLdr, 0 );
			/*
				The brightness scale is split in 8.
															ADC value (LDR) 
				brightness level 0 	-> 	[0 to 32] Less dark
				brightness level 1 	-> 	[33 to 64] 
				brightness level 2 	-> 	[65 to 96]
				brightness level 3 	-> 	[97 to 128]
				brightness level 4 	-> 	[129 to 160]
				brightness level 5 	-> 	[161 to 192]
				brightness level 6 	-> 	[193 to 224]
				brightness level 7 	-> 	[225 to 255] More dark
			
				The mask (& 0x7) certifie that the brightness range is between 0 and 7
			*/
			brightness = (valueLdr/33) & 0x7;
			led.toggleRed();
			static CSensors* Sensors = CSensors::getInstance(); //static because the pointer just can be initialized once
		
			/*Lock Mutex*/
			if( xSemaphoreTake( mutexSensors, ( TickType_t ) 10 ) == pdTRUE )
        {
          Sensors->setDataLdr(brightness);  
					/*Unlock Mutex*/
					xSemaphoreGive( mutexSensors );
        }				
		}
		vTaskDelay(1); //context switch
	}
}

/*******************************************************************************
* Function Name  : vProcessDataTask
* Description    : call function of the task Process Data Task
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vProcessDataTask(void *pvParameters )
{
	CMicrophone *micro = CMicrophone::getInstance();
	micro->initMicrophone();
	for( ;; )
	{	
	micro->readMicrophone(); 
	vTaskDelay(1); //context switch
	}
}

/*******************************************************************************
* Function Name  : vMakeGraphTask
* Description    : call function of the task Make Graph Task
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vMakeGraphTask(void *pvParameters)
{
	extern xQueueHandle Queue_DataMining_Make; // falta fazer o xQueueCreate (CLecs::InitQueue acima)
	extern SemaphoreHandle_t mutex3DPattern;  //mutex 3D Pattern 
	extern  uint16_t spectrum[64];
	extern uint16_t maxValue;
	
	C3DLedMatrixBuffer* buffer = C3DLedMatrixBuffer::getInstance();
	C3DLedMatrix* matrix3D = new C3DLedMatrix;
	
	char*** _3Dmatrix;
	_3Dmatrix = new char**[__LAYERS];  //allocate a pointer to 2D matrixs
	for (int i = 0; i < __LAYERS; ++i)
	{
		_3Dmatrix[i] = new char*[__ROWS](); //allocate a pointer to columns
		for (int j = 0; j < __ROWS; ++j)
		{
			_3Dmatrix[i][j] = new char[__COLUMNS](); // allocate and set all elements 0
			
		}
	}
		
	char bit = 0;
	
	/*effect will coming of the task data mining (microphone) */
	uint16_t effect = 3, x, i;
	
	for(;;)
	{
		if(uxQueueMessagesWaiting(Queue_DataMining_Make))
		{ 
			xQueueReceive( Queue_DataMining_Make, &effect , 0 );
			
		}
		else
		{
	/*********************ALGORITHM's for make patterns*************************************************/
		
		/*Lock Mutex*/
		if( xSemaphoreTake( mutex3DPattern, ( TickType_t ) 0 ) == pdTRUE )
			{
				switch (effect)
				{
					case 1: //ascendant effect
						x = maxValue/ 5;
				
						for (int index = 0, k = 0; index < 50; index++)
						{	index++;
							i = spectrum[index] / x;
							(spectrum[index] > 0) ? bit = 1 : bit = 0;
							(i >= 0) ? _3Dmatrix[0][index / 10][k] = bit : _3Dmatrix[0][index / 10][k] = 0;
							(i >= 1) ? _3Dmatrix[1][index / 10][k] = bit : _3Dmatrix[1][index / 10][k] = 0;
							(i >= 2) ? _3Dmatrix[2][index / 10][k] = bit : _3Dmatrix[2][index / 10][k] = 0;
							(i >= 3) ? _3Dmatrix[3][index / 10][k] = bit : _3Dmatrix[3][index / 10][k] = 0;
							(i >= 4) ? _3Dmatrix[4][index / 10][k] = bit : _3Dmatrix[4][index / 10][k] = 0;
							if(++k > 4) k = 0;
						} 
						break;
					case 2: // X effect
						x = maxValue/ 5;
				
						for (int index = 0, k = 0; index < 50; index++)
						{	index++;
							i = spectrum[index] / x;
							(spectrum[index] > 0) ? bit = 1 : bit = 0;
							(i >= 0) ? _3Dmatrix[k][index / 10][k] 		= bit : _3Dmatrix[k][index / 10][k] 			= 0;
							(i >= 0) ? _3Dmatrix[k][index / 10][4 - k] = bit : _3Dmatrix[k][index / 10][4 - k] 	= 0;
							(i >= 1) ? _3Dmatrix[k][index / 10][k]			= bit : _3Dmatrix[k][index / 10][k]		 	= 0;
							(i >= 1) ? _3Dmatrix[k][index / 10][4 - k] = bit : _3Dmatrix[k][index / 10][4 - k] 	= 0;
							(i >= 2) ? _3Dmatrix[k][index / 10][k] 		= bit : _3Dmatrix[k][index / 10][k] 			= 0;
							(i >= 2) ? _3Dmatrix[k][index / 10][4 - k] = bit : _3Dmatrix[k][index / 10][4 - k] 	= 0;
							(i >= 3) ? _3Dmatrix[k][index / 10][k] 		= bit : _3Dmatrix[k][index / 10][k] 			= 0;
							(i >= 3) ? _3Dmatrix[k][index / 10][4 - k] = bit : _3Dmatrix[k][index / 10][4 - k] 	= 0;
							(i >= 4) ? _3Dmatrix[k][index / 10][k] 		= bit : _3Dmatrix[k][index / 10][k]				= 0;
							(i >= 4) ? _3Dmatrix[k][index / 10][4 - k] = bit : _3Dmatrix[k][index / 10][4 - k] 	= 0;
							if(++k > 4) k = 0;
						}
						break;
					case 3: // ladder effect
						x = maxValue/ 5;
				
						for (int index = 0, k = 0; index < 50; index++)
						{	index++;
							i = spectrum[index] / x;
							(spectrum[index] > 0) ? bit = 1 : bit = 0;
							((i >= 0)&&((index / 10)>=0)) ? _3Dmatrix[0][4 - index / 10][k] = bit : _3Dmatrix[0][4 - index / 10][k] = 0;
							((i >= 1)&&((index / 10)>=1)) ? _3Dmatrix[1][4 - index / 10][k] = bit : _3Dmatrix[1][4 - index / 10][k] = 0;
							((i >= 2)&&((index / 10)>=2)) ? _3Dmatrix[2][4 - index / 10][k] = bit : _3Dmatrix[2][4 - index / 10][k] = 0;
							((i >= 3)&&((index / 10)>=3)) ? _3Dmatrix[3][4 - index / 10][k] = bit : _3Dmatrix[3][4 - index / 10][k] = 0;
							((i >= 4)&&((index / 10)>=4)) ? _3Dmatrix[4][4 - index / 10][k] = bit : _3Dmatrix[4][4 - index / 10][k] = 0;
							if(++k > 4) k = 0;
						} 
						break;
					default:
						break;
					}
					/*Unlock Mutex*/
					xSemaphoreGive( mutex3DPattern );
				}
	/************************************************************************************************************************************/
		
		/*Lock Mutex*/
	if( xSemaphoreTake( mutex3DPattern, ( TickType_t ) 0 ) == pdTRUE )
		{
			
			matrix3D->set3DMatrix(_3Dmatrix);
			buffer->pushFrame(matrix3D);
			
			/*Unlock Mutex*/
			xSemaphoreGive( mutex3DPattern );
		}
	}
	vTaskDelay(1); //context switch	
 }
}


/*******************************************************************************
* Function Name  : vUpdateMatrixTask
* Description    : call function of the task Update Matrix Task
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vUpdateMatrixTask(void *pvParameters)
{
	extern SemaphoreHandle_t Sem_ISR_3D, Sem_ISR_ChangePattern;
	extern SemaphoreHandle_t mutex3DPattern;
	C3DLedMatrixBuffer* buffer = C3DLedMatrixBuffer::getInstance();
	static C3DLedMatrix* matrix3D = new C3DLedMatrix;
		
	for(;;)
	{
		if( xSemaphoreTake( Sem_ISR_ChangePattern, 0 ) == pdTRUE ) // change frame (30 fps)
					{
						/*Lock Mutex*/
						if( xSemaphoreTake( mutex3DPattern, ( TickType_t ) 0 ) == pdTRUE )
						{
							matrix3D = buffer->popFrame();

							/*Unlock Mutex*/
							xSemaphoreGive( mutex3DPattern );}
					}

		if( xSemaphoreTake( Sem_ISR_3D, 0 ) == pdTRUE ) // change layer (1ms)
					{
						matrix3D->write3DMatrix();
					}
		vTaskDelay(1); //context switch
	}
}


/*******************************************************************************
* Function Name  : vDataMiningTask
* Description    : call function of the task Data Mining
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vDataMiningTask(void *pvParameters )
{
	extern xQueueHandle Queue_SensorFusion_DataMining, Queue_DataMining_Make ;
	extern SemaphoreHandle_t Sem_DataMining_Sleep;// When a sleep condition is verify this semaphore is released and the programme go to a sleep mode.
  const int maxTic = 100;
	bool updatePattern = false;
	char effect = 1, obs = 'o';								//guarda valor do observado 'o' = outro; 'd' = direita; 'e' = esquerda;	
	unsigned short int state = 0;	//guarda valor dos estados: 0 = inicio; 1 = começar gesto esq; 2 = começar gesto dir; 3 = gesto esq; 4 = gesto dir;
	unsigned short int tic = 0;	
	static CSensors* Sensors = CSensors::getInstance(); //static because the pointer just can be initialized once
	
	for( ;; )
	{		
		if(uxQueueMessagesWaiting(Queue_SensorFusion_DataMining))
		{ 
			xQueueReceive( Queue_SensorFusion_DataMining, &obs, 0 );
			
			switch (state)
			{
				case 0:
								if(obs == 'd')
									{state = 1;}
								else if(obs == 'e')
									{state = 2;}
								tic = 0;
								break;
				case 1:								
								if(tic >= maxTic)
									{state = 0;}
								else if(obs == 'e')
									{state = 3;}
								tic++;
								break;
				case 2:
								if(tic >= maxTic)
									{state = 0;}
								else if(obs == 'd')
									{state = 4;}
								tic++;
								break;
				case 3:
								//gesto esquerda
								(effect < 2 ) ? effect = 3 : effect--;
								state = 0;
								updatePattern = true;
								break;
				case 4:
								//gesto direita
								(effect > 2 ) ? effect = 1 : effect++;
								state = 0;
								updatePattern = true;
								break;
				default: break;			
			}
			
			if(updatePattern)
			{
				updatePattern = false;
				xQueueSend(Queue_DataMining_Make, &effect ,0);
			}
			
			if(Sensors->getDataLdr() < 2 ) // isn't DARK ?? 
			{
			/* Unblock the task by releasing the semaphore. */
			xSemaphoreGive( Sem_DataMining_Sleep);
			portYIELD();
			}
			
			vTaskDelay(1); //context switch
		}
	}
}
/*******************************************************************************
* Function Name  : vSensorFusionTask
* Description    : call function of the task Sensor Fusion
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vSensorFusionTask(void *pvParameters )
{
	uint8_t data = 0, a, b, c, d;
	extern SemaphoreHandle_t mutexSensors; 
	extern xQueueHandle Queue_SensorFusion_DataMining;//Send the read data from capacitive sensors, LDR sensor and microphone sensor in one only structure of data that was fused.
	char obs;
	CLeds led;
	
	static CSensors* Sensors = CSensors::getInstance(); //static because the pointer just can be initialized once
	
	for( ;; )
	{	
		/*Lock Mutex*/
		if( xSemaphoreTake( mutexSensors, ( TickType_t ) 10 ) == pdTRUE )
			{
				data = Sensors->getDataCapSensors();
				/*Unlock Mutex*/
				xSemaphoreGive( mutexSensors );
			}
		a = (data & 0x01) >> 0; 
		b = (data & 0x02) >> 1; 
		c = (data & 0x04) >> 2; 
		d = (data & 0x08) >> 3;
				
		if((b||d) && !(a||c) )
			{
				obs = 'e';
			}
			else if((a||c) && !(b||d) )
			{

				obs = 'd';
			}
			else
			{	
				obs = 'o';
			}
		xQueueSend(Queue_SensorFusion_DataMining, &obs ,0);
		vTaskDelay(1); //context switch
	}
}
/*******************************************************************************
* Function Name  : vCapSensorTask
* Description    : call function of the task Capacitive Sensors
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vCapSensorTask(void *pvParameters )
{
	extern xQueueHandle Queue_ISR_CapSensor;
	extern SemaphoreHandle_t mutexSensors; 
	uint8_t data;

	for( ;; )
	{		
		if(uxQueueMessagesWaiting(Queue_ISR_CapSensor))
		{ 
			xQueueReceive( Queue_ISR_CapSensor, &data, 0 );
			
			static CSensors* Sensors = CSensors::getInstance(); //static because the pointer just can be initialized once
		
			/*Lock Mutex*/
			if( xSemaphoreTake( mutexSensors, ( TickType_t ) 10 ) == pdTRUE )
        {
          Sensors->setDataCapSensors(data);
					/*Unlock Mutex*/
					xSemaphoreGive( mutexSensors );
        }				
		}
		vTaskDelay(1); //context switch
	}
}

/*******************************************************************************
* Function Name  : TIM6_DAC_IRQHandler
* Description    : ISR timer 6
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
// extern "C" -> for C++, ensure the interrupt handler is linked as a C function
extern "C" void TIM5_IRQHandler(void) 
{
	extern SemaphoreHandle_t Sem_ISR_Sleep;
	
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	static CSensors* Sensors = CSensors::getInstance(); //static because the pointer just can be initialized once
	
	if (TIM_GetITStatus (TIM5, TIM_IT_Update) != RESET) 
	{
		
		if(Sensors->getDataLdr() >= 2) // DARK ?? 
		{
		/* Unblock the task by releasing the semaphore. */
		xSemaphoreGiveFromISR( Sem_ISR_Sleep, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		
		TIM_ClearITPendingBit (TIM5, TIM_IT_Update); 
  }	
}

/*******************************************************************************
* Function Name  : vSleepTask
* Description    : call function of the task Sleep Task
* Input          : None 
* Output         : None 
* Return			   : None
*******************************************************************************/
void vSleepTask(void *pvParameters )
{
	extern TaskHandle_t tMakeGraphTask;
	extern TaskHandle_t tProcessDataTask;
	extern TaskHandle_t tUpdateMatrixTask;
	extern TaskHandle_t tDataMiningTask;
	extern TaskHandle_t tSensorFusionTask;
	extern TaskHandle_t tCapSensorTask;
	extern SemaphoreHandle_t Sem_ISR_Sleep;   		//when a wake up condition is verify this semaphore is released and the programme wake up and start running normally.
	extern SemaphoreHandle_t Sem_DataMining_Sleep;// When a sleep condition is verify this semaphore is released and the programme go to a sleep mode.

	C3DLedMatrix* matrix3D = new C3DLedMatrix;
	
	char*** _3Dmatrix;
	_3Dmatrix = new char**[__LAYERS];  //allocate a pointer to 2D matrixs
	for (int i = 0; i < __LAYERS; ++i)
	{
		_3Dmatrix[i] = new char*[__ROWS](); //allocate a pointer to columns
		for (int j = 0; j < __ROWS; ++j)
		{
			_3Dmatrix[i][j] = new char[__COLUMNS](); // allocate and set all elements 0
			
		}
	}
	
	CTimer timer5; 
	
	/*Interrupt in 1s in 1s*/
	timer5.timerInit(42000, 1000, RCC_APB1Periph_TIM5, TIM5);
	timer5.timerInterruptInit(TIM5_IRQn);
	timer5.timerInterruptEnable(TIM5);
	
	for( ;; )
	{	
		if(uxSemaphoreGetCount( Sem_DataMining_Sleep ) != 0) // Sleep Mode
		{
			matrix3D->write3DMatrix(); // turn off 3d matrix of leds 
						
			vTaskSuspend( tMakeGraphTask );
			vTaskSuspend( tProcessDataTask );
			vTaskSuspend( tUpdateMatrixTask );
			vTaskSuspend( tDataMiningTask );
			vTaskSuspend( tSensorFusionTask );
			vTaskSuspend( tCapSensorTask );
			
			timer5.timerStart(TIM5); // timer to verify if is dark or no
		
			xSemaphoreTake( Sem_DataMining_Sleep, 0 );
		}
		
		if(uxSemaphoreGetCount( Sem_ISR_Sleep ) != 0) // Wake Up Mode
		{
			timer5.timerStop(TIM5);
			
			vTaskResume(tMakeGraphTask);
			vTaskResume(tProcessDataTask);
			vTaskResume(tUpdateMatrixTask);
			vTaskResume(tDataMiningTask);
			vTaskResume(tSensorFusionTask);
			vTaskResume(tCapSensorTask);
						
			xSemaphoreTake( Sem_ISR_Sleep, 0 );
		}

		vTaskDelay(1); //context switch
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
	extern TaskHandle_t tMakeGraphTask;
	extern TaskHandle_t tProcessDataTask;
	extern TaskHandle_t tUpdateMatrixTask;
	extern TaskHandle_t tDataMiningTask;
	extern TaskHandle_t tSensorFusionTask;
	extern TaskHandle_t tCapSensorTask;
	extern TaskHandle_t tLDRTask;
	extern TaskHandle_t tSleepTask;
	portBASE_TYPE updateMatrixTask, makeGraphTask, processDataTask, LDRTask, sleepTask, capSensorTask, sensorFusionTask, dataMiningTask ;
	
	/* Create Task */
	/*Low priority numbers denote low priority tasks. The idle task has priority zero (tskIDLE_PRIORITY).*/
	makeGraphTask = xTaskCreate(vMakeGraphTask, "makeGraphTask", configMINIMAL_STACK_SIZE, NULL, 4, &tMakeGraphTask); // priority higher than 4 don't work
	processDataTask = xTaskCreate(vProcessDataTask, "processDataTask", configMINIMAL_STACK_SIZE, NULL, 4, &tProcessDataTask);
	updateMatrixTask = xTaskCreate(vUpdateMatrixTask, "updateMatrixTask", configMINIMAL_STACK_SIZE, NULL, 3, &tUpdateMatrixTask);
	dataMiningTask = xTaskCreate(vDataMiningTask, "dataMiningTask", configMINIMAL_STACK_SIZE, NULL, 2, &tDataMiningTask);
	sensorFusionTask = xTaskCreate(vSensorFusionTask, "sensorFusionTask", configMINIMAL_STACK_SIZE, NULL, 2, &tSensorFusionTask);
	capSensorTask = xTaskCreate(vCapSensorTask, "capSensorTask", configMINIMAL_STACK_SIZE, NULL, 1, &tCapSensorTask);
	LDRTask = xTaskCreate(vLDRTask, "LDRTask", configMINIMAL_STACK_SIZE, NULL, 0, &tLDRTask);
	sleepTask = xTaskCreate(vSleepTask, "sleepTask", configMINIMAL_STACK_SIZE, NULL, 0, &tSleepTask);

//	if ((updateMatrixTask == pdPASS)&&(makeGraphTask == pdPASS)&&(processDataTask == pdPASS)&&(LDRTask == pdPASS)&&(sleepTask == pdPASS)&&(capSensorTask == pdPASS)&&(sensorFusionTask == pdPASS)&&(dataMiningTask == pdPASS))
//	{
//		/* Everything went well*/
//		return 0;
//	}
//	else
//	{
//		/* ERROR! Creating the Tasks */
//		return -2;
//	}
return 0;
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

/*
	Overload of opeartor new and delete because they use malloc and free
that aren not thread safe
*/
void *operator new(size_t size)
{
   void *p;

   if(uxTaskGetNumberOfTasks())
      p=pvPortMalloc(size); //thread safe
   else
      p=malloc(size); //no thread safe, just when we have only one thread

  return p;
}

/*
	Overload of opeartor new and delete because they use malloc and free
that aren not thread safe
*/
void operator delete(void *p)
{
   if(uxTaskGetNumberOfTasks())
      vPortFree( p );//thread safe
   else
      free( p );//no thread safe, just when we have only one thread

   p = NULL;

}
