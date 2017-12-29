#include "CMicrophone.h"
#include "stm32f4xx.h"
#include "pdm_filter.h"

#define __AUDIO_IN_FREQ   				 	16 // 16k
#define __DECIMATION_FACTOR       	64
#define __SAMPLE_FREQUENCY					16000	//For 16kHz
#define __INPUT_CHANNELS 						2
#define __OUT_FREQ              	  __SAMPLE_FREQUENCY/2
#define __PDM_Input_Buffer_SIZE  		(__OUT_FREQ / 1000 *__DECIMATION_FACTOR*__INPUT_CHANNELS/8 )	//(128*OUT_FREQ/DEFAULT_AUDIO_IN_FREQ *INPUT_CHANNELS)//
#define __PCM_Output_Buffer_SIZE 		(__OUT_FREQ / 1000 *__INPUT_CHANNELS)
#define __SAMPLES										64			/*More samples more recorded quality */ 
#define __FFT_SIZE									__SAMPLES//__SAMPLES/2
#define __Buffer_Input_SIZE					__SAMPLES	//PCM_Output_Buffer_SIZE  * OUT_FREQ/(_AUDIO_IN_FREQ/2)

/* SPI Configuration defines */
#define __SPI_SCK_PIN               GPIO_Pin_10
#define __SPI_SCK_GPIO_PORT         GPIOB
#define __SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define __SPI_SCK_SOURCE            GPIO_PinSource10
#define __SPI_SCK_AF                GPIO_AF_SPI2

#define __SPI_MOSI_PIN              GPIO_Pin_3
#define __SPI_MOSI_GPIO_PORT        GPIOC
#define __SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOC
#define __SPI_MOSI_SOURCE           GPIO_PinSource3
#define __SPI_MOSI_AF               GPIO_AF_SPI2


uint16_t PDM_Input_Buffer[__PDM_Input_Buffer_SIZE]; // buffer for PDM samples
uint16_t PCM_Output_Buffer[__PCM_Output_Buffer_SIZE]; // 1st buffer for PCM samples
uint16_t PCM_Output_Buffer1[__PCM_Output_Buffer_SIZE]; // 2nd buffer for PCM samples(used when data from first is being saved)
uint16_t buffer_input[__Buffer_Input_SIZE]; // 1st buffer that aggregates PCM samples
uint16_t buffer_input1[__Buffer_Input_SIZE]; // 2nd buffer that aggregates PCM samples(used when the first is busy)
uint16_t spectrum[__FFT_SIZE];
uint16_t spectrum1[__FFT_SIZE];

uint16_t buf_idx = 0, buf_idx1 =0;
uint16_t* AudioRecBuf; // pointer to Audio Recording Buffer, it's set to PCM_Output_Buffer or PCM_Output_Buffer2
uint16_t* WriteBuf; // pointer to buffer, from which bytes are being sent
uint16_t maxValue; //for  FFT
uint16_t counter = 0;
uint8_t Switch = 0;
uint32_t Data_Status =0; // ------------------------------------------temos de colocar um semaphore ----------------------------------------------------------------------------------
uint32_t AudioRecInited = 0;/* Current state of the audio recorder interface intialization */
uint32_t InternalBufferSize = 0;

PDMFilter_InitStruct Filter;

arm_rfft_fast_instance_f32 S;


/* Private function prototypes -----------------------------------------------*/
//static void GPIO_Init(void);
//static void SPI_Init(uint32_t Freq);
//static void NVIC_Init(void);
//uint32_t WaveRecorderInit(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);


/*******************************************************************************
* Function Name  : 
* Description    : This function handles AUDIO_REC_SPI global interrupt request.
* Input          : None (void)
* Output         : None (void)
* Return				 : None (void)
*******************************************************************************/
extern "C" void SPI2_IRQHandler(void)
{  
  u16 volume; u16 app;
	extern SemaphoreHandle_t Sem_ISR_ProcessData; 	//Send the read data from Microphone sensor to Process Data Task.
	BaseType_t xHigherPriorityTaskWoken;
	/* We have not woken a task at the start of the ISR. */
	xHigherPriorityTaskWoken = pdFALSE;
	
	static int i = 0;
	if(i++ == 5000){
	GPIO_ToggleBits(GPIOD, GPIO_Pin_13); i = 0;}
	
	
  /* Check if data are available in SPI Data register */
  if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
  {
    app = SPI_I2S_ReceiveData(SPI2);
    PDM_Input_Buffer[InternalBufferSize++] = (HTONS(app));
    
    /* Check to prevent overflow condition */
    if (InternalBufferSize >= __PDM_Input_Buffer_SIZE)
    {
      InternalBufferSize = 0;
     
      volume = 50;
      
      PDM_Filter_64_LSB((uint8_t *)PDM_Input_Buffer, (uint16_t *)AudioRecBuf, volume , (PDMFilter_InitStruct *)&Filter);
 			xSemaphoreGiveFromISR( Sem_ISR_ProcessData, &xHigherPriorityTaskWoken );
    }
  }
}

CMicrophone::CMicrophone()
{
}


CMicrophone::~CMicrophone()
{
}

void CMicrophone::initMicrophone()
{
	waveRecorderInit(__AUDIO_IN_FREQ*1000,16, __INPUT_CHANNELS);
}

void CMicrophone::closeMicrophone()
{
}
void WaveRecorderUpdate(void);

int CMicrophone::readMicrophone()
{
	waveRecorderUpdate();
	return 0;
}

CMicrophone* CMicrophone::instance = 0;

CMicrophone * CMicrophone::getInstance()
{
	//mutex lock //Thread Safe Singleton, avoid race condition during the initialization of the static Singleton
	if (instance == 0)
		instance = new CMicrophone;
	//mutex unclock 
	return instance;
}

/*******************************************************************************
* Function Name  : GPIOInit
* Description    : Initialize GPIO for wave recorder.
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CMicrophone::GPIOInit(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(__SPI_SCK_GPIO_CLK | __SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(__SPI_SCK_GPIO_CLK | __SPI_MOSI_GPIO_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = __SPI_SCK_PIN;
  GPIO_Init(__SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
  
  /* Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(__SPI_SCK_GPIO_PORT, __SPI_SCK_SOURCE, __SPI_SCK_AF);
  
  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  __SPI_MOSI_PIN;
  GPIO_Init(__SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(__SPI_MOSI_GPIO_PORT, __SPI_MOSI_SOURCE, __SPI_MOSI_AF);
}

/*******************************************************************************
* Function Name  : SPIInit
* Description    : Initialize SPI peripheral.
* Input          :	Freq :Audio frequency
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CMicrophone::SPIInit(uint32_t Freq)
{
  I2S_InitTypeDef I2S_InitStructure;

  /* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  
  /* SPI configuration */
  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = Freq * 2; 
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);

  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	
	RCC_PLLI2SConfig(258,3); 														// ------------------------------------------------
	
	RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);					// ------------------------------------------------
	RCC_PLLI2SCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY)==RESET);// ------------------------------------------------
	
}

/*******************************************************************************
* Function Name  : NVICInit
* Description    : Initialize the NVIC.
* Input          : None (void)
* Output         : None (void)
* Return				 : None
*******************************************************************************/
void CMicrophone::NVICInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}




/*******************************************************************************
* Function Name  : WaveRecorderInit
* Description    : Initialize wave recording
* Input          : AudioFreq: Sampling frequency
*								 : BitRes: Audio recording Samples format (from 8 to 16 bits)
*								 : ChnlNbr: Number of input microphone channel
* Output         : uint32_t
* Return				 : 0 -> Return 0 if all operations are OK 
*******************************************************************************/
uint32_t CMicrophone::waveRecorderInit(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{ 
  /* Check if the interface is already initialized */
  if (AudioRecInited)
  {
    /* No need for initialization */
    return 0;
  }
  else
  {
    /* Enable CRC module */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    
    /* Filter LP & HP Init */
    Filter.LP_HZ = __OUT_FREQ;
    Filter.HP_HZ = 10;
    Filter.Fs = __SAMPLE_FREQUENCY;
    Filter.Out_MicChannels = 1;
    Filter.In_MicChannels = 1;
    
    PDM_Filter_Init((PDMFilter_InitStruct *)&Filter);
		
		// Initialize FFT for 32 samples
		arm_rfft_fast_init_f32(&S, __FFT_SIZE );
		    
    /* Configure the GPIOs */
    GPIOInit();
    
    /* Configure the interrupts (for timer) */
    NVICInit();
    
    /* Configure the SPI */
    SPIInit(AudioFreq);
		
    /* Set the local parameters */
//    AudioRecBitRes = BitRes;
//    AudioRecChnlNbr = ChnlNbr;
    
    /* Set state of the audio recorder to initialized */
    AudioRecInited = 1;
    
    /* Return 0 if all operations are OK */
    return 0;
  }  
}



/*******************************************************************************
* Function Name  : waveRecorderStart
* Description    : Start audio recording
* Input          : pbuf: pointer to a buffer
*								 : size: Buffer size
* Output         : uint8_t
* Return				 : 0 -> Return 0 if all operations are OK 
*								 : 1 -> Cannot perform operation 
*******************************************************************************/
uint8_t CMicrophone::waveRecorderStart(uint16_t* pbuf, uint32_t size)
{
/* Check if the interface has already been initialized */
  if (AudioRecInited)
  {
    /* Store the location and size of the audio buffer */
    AudioRecBuf = pbuf;
    
    /* Enable the Rx buffer not empty interrupt */
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    /* The Data transfer is performed in the SPI interrupt routine */
    /* Enable the SPI peripheral */
    I2S_Cmd(SPI2, ENABLE); 
   
    /* Return 0 if all operations are OK */
    return 0;
  }
  else
  {
    /* Cannot perform operation */
    return 1;
  }
}


/*******************************************************************************
* Function Name  : waveRecorderStop
* Description    : Stop audio recording
* Input          : None (void)
* Output         : uint32_t
* Return				 : 0 -> Return 0 if all operations are OK 
*								 : 1 -> Cannot perform operation 
*******************************************************************************/
uint32_t CMicrophone::waveRecorderStop(void)
{
  /* Check if the interface has already been initialized */
  if (AudioRecInited)
  {
    
    /* Stop conversion */
    I2S_Cmd(SPI2, DISABLE); 
    
    /* Return 0 if all operations are OK */
    return 0;
  }
	
	
	
  else
  {
    /* Cannot perform operation */
    return 1;
  }
}


/*******************************************************************************
* Function Name  : maxArray
* Description    : Calculate the max of one array
* Input          : uint16_t a[], uint16_t num_elements
* Output         : Output
* Return				 : max
*******************************************************************************/
uint16_t CMicrophone::maxArray(uint16_t a[], uint16_t num_elements)
{
   int i, max=-32000;
   for (i=0; i<num_elements; i++)
   {
	 if (a[i]>max)
	 {
	    max=a[i];
	 }
   }
   return(max);
}
/*******************************************************************************
* Function Name  : waveRecorderUpdate
* Description    : Update the recorded data 
* Input          : None (void)
* Output         : None (void)
* Return				 : None (void)
*******************************************************************************/
void CMicrophone::waveRecorderUpdate(void)
{     
  /* Start the record */
	static int i = 0;
	extern SemaphoreHandle_t Sem_ISR_ProcessData; 	//Send the read data from Microphone sensor to Process Data Task.

	if(!i)
  waveRecorderStart(PCM_Output_Buffer, __PCM_Output_Buffer_SIZE); i=1;
	
	/* Wait for the data to be ready with PCM form */
	if( xSemaphoreTake( Sem_ISR_ProcessData, 0 ) == pdTRUE ) 
		{
			/* Switch the buffers*/
			if (Switch ==1)
			{
			 AudioRecBuf = PCM_Output_Buffer;
			 WriteBuf = PCM_Output_Buffer1;
			 Switch = 0;
			}
			else
			{
				AudioRecBuf = PCM_Output_Buffer1;
				WriteBuf = PCM_Output_Buffer;
				Switch = 1;
			}
				
			for (counter=0; counter<16; counter++)
			{
				if (buf_idx< __Buffer_Input_SIZE)
				{
					/* Store Data in RAM buffer */
					buffer_input[buf_idx++]= *(WriteBuf + counter);
					if (buf_idx1 == __Buffer_Input_SIZE)
					{
						buf_idx1 = 0;
						/*FFT second half of the buffer*/			
						arm_rfft_fast_f32(&S,(float32_t*)buffer_input1, (float32_t*)spectrum, 0);  //do the fft of the signal recorded by the micro
						arm_cmplx_mag_f32((float32_t*)spectrum, (float32_t*)spectrum, __FFT_SIZE); //calcule the magnitude of each freq
						maxValue = maxArray(spectrum, __FFT_SIZE);//Calcule the max of the spectrum
					}
				}
				else 
					if (buf_idx1< __Buffer_Input_SIZE)
					{
						static int fftDone = 1;
						/* Store Data in RAM buffer */
						buffer_input1[buf_idx1++]= *(WriteBuf + counter);
						if ((buf_idx == __Buffer_Input_SIZE) && fftDone)
						{
							/*FFT first half of the buffer*/
							arm_rfft_fast_f32(&S,(float32_t*)buffer_input,(float32_t*)spectrum, 0);//do the fft of the signal recorded by the micro
							arm_cmplx_mag_f32((float32_t*)spectrum, (float32_t*)spectrum, __FFT_SIZE); //calcule the magnitude of each freq
							maxValue = maxArray(spectrum, __FFT_SIZE); //Calcule the max of the spectrum
							fftDone = 0; //indicate that FFT already was done
						}
						if (buf_idx1 == __Buffer_Input_SIZE)
								{
									buf_idx = 0;
									fftDone = 1;
								}
					} 
			 }
		}
}

