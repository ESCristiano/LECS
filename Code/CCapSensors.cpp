#include "CCapSensors.h"
#include <stm32f4xx.h>
#include "CTimer.h"
#include "CBoardLeds.h"

CCapSensors::CCapSensors()
{
}


CCapSensors::~CCapSensors()
{
}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : ISR timer 4
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
// extern "C" -> for C++, ensure the interrupt handler is linked as a C function
extern "C" void   TIM4_IRQHandler(void) 
{ 
	CLeds led;
	extern xQueueHandle Queue_ISR_CapSensor;					 //Send the read data from capacitive sensors to Cap Sensor Task.
	BaseType_t xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	uint8_t data;
	static CCapSensors *cap = CCapSensors::getInstance();
	
	if (TIM_GetITStatus (TIM4, TIM_IT_Update) != RESET) {
		data = cap->readCapSensors();
		xQueueSendFromISR(Queue_ISR_CapSensor, &data, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    TIM_ClearITPendingBit (TIM4, TIM_IT_Update); 
  }
}

void CCapSensors::initCapacitiveSensor()
{
		/*input for capacitive Sensor*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct;
	// GPIO peripheral properties specification
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // define the GPIO with input
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN; // pulldown resistors 
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
			/*input for capacitive Sensor*/
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct_1;
	// GPIO peripheral properties specification
	GPIO_InitStruct_1.GPIO_Pin = GPIO_Pin_12; 
	GPIO_InitStruct_1.GPIO_Mode = GPIO_Mode_IN; // define the GPIO with input
	GPIO_InitStruct_1.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct_1.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct_1.GPIO_PuPd = GPIO_PuPd_DOWN; // pulldown resistors 
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOC, &GPIO_InitStruct_1);
	
	CTimer timer4; //basic timer4
	
	/*Interrupt in 1ms in 100ms*/
	timer4.timerInit(42000, 200, RCC_APB1Periph_TIM4, TIM4);
	timer4.timerInterruptInit(TIM4_IRQn);
	timer4.timerInterruptEnable(TIM4);
	timer4.timerStart(TIM4);
}

void CCapSensors::closeCapacitiveSensor()
{
}

uint8_t CCapSensors::readCapSensors()
{
	uint8_t data = 0;
	
	data = data | (0x01 & (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) << 0));
	data = data | (0x02 & (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) << 1));
	data = data | (0x04 & (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) << 2));
	data = data | (0x08 & (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) << 3));

	return data;
}

CCapSensors* CCapSensors::instance = 0;

CCapSensors * CCapSensors::getInstance()
{
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton
	if (instance == 0)
		instance = new CCapSensors;
	//mutex unclock 
	return instance;
}

