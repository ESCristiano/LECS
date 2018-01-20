#include "CLightSensor.h"

CLightSensor::CLightSensor()
{
}


CLightSensor::~CLightSensor()
{
}

/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : ISR timer 7
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
// extern "C" -> for C++, ensure the interrupt handler is linked as a C function
extern "C" void TIM7_IRQHandler(void) 
{
	uint16_t value;
	extern xQueueHandle Queue_ISR_LDR; extern SemaphoreHandle_t Sem_DataMining_Sleep;
	BaseType_t xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	
	static CLightSensor* ldr = CLightSensor::getInstance();
	if (TIM_GetITStatus (TIM7, TIM_IT_Update) != RESET) {
    value = ldr->readLDR(); 
		xQueueSendFromISR(Queue_ISR_LDR, &value, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    TIM_ClearITPendingBit (TIM7, TIM_IT_Update); 
  }
}


void CLightSensor::initLightSensor()
{
	this->initLDR();
	timer7.timerInit(42000, 2000, RCC_APB1Periph_TIM7, TIM7);
	timer7.timerInterruptInit(TIM7_IRQn);
	timer7.timerInterruptEnable(TIM7);
	timer7.timerStart(TIM7);

}

/*******************************************************************************
* Function Name  : closeLDR
* Description    : Cleanup
* Input          : None (void)
* Output         : None (void)
* Return	    	 : None
*******************************************************************************/
void CLightSensor::closeLDR()
{
	timer7.timerStop(TIM7);
	timer7.timerInterruptDisable(TIM7);
}

/*******************************************************************************
* Function Name  : readLDR
* Description    : Initialization of the adc for read LDR
* Input          : None (void)
* Output         : uint16_t
* Return				 : Value between 0 and 255 that represent the value of LDR
*******************************************************************************/
uint16_t CLightSensor::readLDR()
{
	ADC_SoftwareStartConv(ADC1);//Start the conversion
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	return (ADC_GetConversionValue(ADC1) & 0x00ff); //Return the converted data
}

CLightSensor* CLightSensor::instance = 0;

CLightSensor * CLightSensor::getInstance()
{
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton
	if (instance == 0)
		instance = new CLightSensor;
	//mutex unclock 
	return instance;
}

/*******************************************************************************
* Function Name  : initLDR
* Description    : Initialization of the adc 1 channel 1 PA1 for read LDR
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLightSensor::initLDR()
{
	/**************************ADC_InitTypeDef***********************************+
	uint32_t ADC_Resolution;                !< Configures the ADC resolution dual mode.
	This parameter can be a value of @ref ADC_resolution
	FunctionalState ADC_ScanConvMode;       !< Specifies whether the conversion
	is performed in Scan (multichannels)
	or Single (one channel) mode.
	This parameter can be set to ENABLE or DISABLE
	FunctionalState ADC_ContinuousConvMode; !< Specifies whether the conversion
	is performed in Continuous or Single mode.
	This parameter can be set to ENABLE or DISABLE.
	uint32_t ADC_ExternalTrigConvEdge;      !< Select the external trigger edge and
	enable the trigger of a regular group.
	This parameter can be a value of
	ADC_external_trigger_edge_for_regular_channels_conversion
	uint32_t ADC_ExternalTrigConv;          !< Select the external event used to trigger
	the start of conversion of a regular group.
	This parameter can be a value of
	ADC_extrenal_trigger_sources_for_regular_channels_conversion
	uint32_t ADC_DataAlign;                 !< Specifies whether the ADC data  alignment
	is left or right. This parameter can be
	a value of ADC_data_align
	uint8_t  ADC_NbrOfConversion;           !< Specifies the number of ADC conversions
	that will be done using the sequencer for
	regular channel group.
	This parameter must range from 1 to 16.
	*/

	/*Enable the ADC interface clock using */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/*ADC pins configuration*/
	GPIO_InitTypeDef GPIO_InitStruct;		// GPIO structure declaration
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // Enabling GPIO peripheral clock
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; // Analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	//ADC structure configuration
	ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
	ADC_DeInit();
	ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
	ADC_init_structure.ADC_Resolution = ADC_Resolution_8b;//Input voltage is converted into a 8bit number giving a maximum value of 256
	ADC_init_structure.ADC_ContinuousConvMode = DISABLE; //the conversion is single mode, the input data is converted once
	ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
	ADC_init_structure.ADC_NbrOfConversion = 1; //number of ADC conversions
	ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
	ADC_Init(ADC1, &ADC_init_structure);//Initialize ADC with the previous configuration
										//Enable ADC conversion
	ADC_Cmd(ADC1, ENABLE);
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
}
