#include "CLdrSensor.h"

CLdrSensor::CLdrSensor()
{
}


CLdrSensor::~CLdrSensor()
{
}

/*******************************************************************************
* Function Name  : initTimer
* Description    : Initialization timer 7 and her interrupt
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::initTimer(void)
{ 
	this->timerInit();
	this->timerInterruptInit();
	this->timerInterruptEnable();
	this->timerStart();
}

/*******************************************************************************
* Function Name  : initLDR
* Description    : Initialization of the adc 1 channel 1 PA1 for read LDR
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::initLDR(void)
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
                                               @ref ADC_external_trigger_edge_for_regular_channels_conversion 
  uint32_t ADC_ExternalTrigConv;          !< Select the external event used to trigger 
                                               the start of conversion of a regular group.
                                               This parameter can be a value of 
                                               @ref ADC_extrenal_trigger_sources_for_regular_channels_conversion 
  uint32_t ADC_DataAlign;                 !< Specifies whether the ADC data  alignment
                                               is left or right. This parameter can be 
                                               a value of @ref ADC_data_align 
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
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; // Analog mode
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	//ADC structure configuration
	ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
	ADC_DeInit();
	ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
	ADC_init_structure.ADC_Resolution = ADC_Resolution_8b;//Input voltage is converted into a 8bit number giving a maximum value of 4096
	ADC_init_structure.ADC_ContinuousConvMode = DISABLE; //the conversion is single mode, the input data is converted once
	ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
	ADC_init_structure.ADC_NbrOfConversion = 1; //number of ADC conversions
	ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
	ADC_Init(ADC1, &ADC_init_structure);//Initialize ADC with the previous configuration
	//Enable ADC conversion
	ADC_Cmd(ADC1, ENABLE);
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_144Cycles);
	
}

/*******************************************************************************
* Function Name  : readLDR
* Description    : Initialization of the adc for read LDR
* Input          : None (void)
* Output         : uint16_t
* Return				 : Value between 0 and 255 that represent the value of LDR
*******************************************************************************/
uint16_t CLdrSensor::readLDR(void)
{
	ADC_SoftwareStartConv(ADC1);//Start the conversion
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	return ADC_GetConversionValue(ADC1); //Return the converted data
}

/*******************************************************************************
* Function Name  : closeLDR
* Description    : Cleanup
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::closeLDR(void)
{
	this->timerStop();
	this->timerInterruptDisable();
}

/*******************************************************************************
* Function Name  : timerInterruptInit
* Description    : Init timer 7 overflow interrupt
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::timerInterruptInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
  /* Enable the timer global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15; //lowest priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;//lowest priority
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : timerInit
* Description    : Init timer 7
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::timerInit(void)
{
	/*
	TIM7 Clock Frequency = 84MHZ
	Prescaler = 42000
	(TIM7 Clock Frequency) / Prescaler = 2000 HZ
	New Frequency = 2000 HZ => ( 0.5 ms for tick )
	To have 1 second = 0.5 ms * 2000
	So, put 2000 in reload.
	*/
	
  uint16_t prescaler = 42000 - 1;
  uint16_t reload = (2000) - 1; //overflow 1 second in 1 second
  
  /*Enable the clock */
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM7, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  /* set everything back to default values */
  TIM_TimeBaseStructInit (&TIM_TimeBaseStructure);
  /* only changes from the defaults are needed */
  TIM_TimeBaseStructure.TIM_Period = reload;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseInit (TIM7, &TIM_TimeBaseStructure);
}

/*******************************************************************************
* Function Name  : timerStart
* Description    : Run timer 7
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::timerStart(void)
{
	TIM_Cmd (TIM7, ENABLE);
}

/*******************************************************************************
* Function Name  : timerStop
* Description    : Stop Timer 7
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::timerStop(void)
{
	TIM_Cmd (TIM7, DISABLE);
}

/*******************************************************************************
* Function Name  : timerInterruptEnable
* Description    : Enable timer 7 interreput
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::timerInterruptEnable(void)
{
	/*
   * It is important to clear any pending interrupt flags since the timer
   * has been free-running since we last used it and that will generate
   * interrupts on overflow even though the associated interrupt event has
   * not been enabled.
   */
  TIM_ClearITPendingBit (TIM7, TIM_IT_Update);
  /* put the counter into a known state */
  TIM_SetCounter (TIM7, 0);
  TIM_ITConfig (TIM7, TIM_IT_Update, ENABLE);
}

/*******************************************************************************
* Function Name  : timerInterruptDisable
* Description    : Disable timer 7 interreput
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CLdrSensor::timerInterruptDisable(void)
{
	TIM_ITConfig (TIM7, TIM_IT_Update, DISABLE);
}
