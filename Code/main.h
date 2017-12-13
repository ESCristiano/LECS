/***************************************************** LECS **********************************************************************************************************
 * by Cristiano Rodrigues and Ivo Marques
 *
 * Project features:
 *
 * - Adapt the effects to the environment based on data received by the sensors using STM32F4 Discovery;
 * - Produce 3D effects in a matrix of leds;
 * - Recognize some gestures;
 * - Recognize diferent levels of luminosity;
 * - Record sounds of the environment;
 * - Detect low and high frequencies (bass and treble) based in analyses the sound captured by microphone;
 * - Standby mode to save energy (when the luminosity is too high, LEDS power off).
 *
 *********************************************************************************************************************************************************************/
#ifndef _MAIN_H
#define _MAIN_H

#include "freertos.h"

/* Library includes. */
#include <stm32f4xx.h>

/********************************************************************************************************************************************************************/

SemaphoreHandle_t Sem_ISR_3D;  		//When an interrupt occur this semaphore is released and the Update Matrix Task, leave your state of wait, and execute your function.
SemaphoreHandle_t Sem_ISR_ChangePattern; //When an interrupt occur this semaphore is released and the change pattern, leave your state of wait, and execute your function.
SemaphoreHandle_t Sem_ISR_Sleep;   //when a wake up condition is verify this semaphore is released and the programme wake up and start running normally.
SemaphoreHandle_t Sem_DataMining_Sleep;// When a sleep condition is verify this semaphore is released and the programme go to a sleep mode.

/********************************************************************************************************************************************************************/

xQueueHandle Queue_ISR_CapSensor;					 //Send the read data from capacitive sensors to Cap Sensor Task.
xQueueHandle Queue_ISR_LDR;      					 //Send the read data from LDR sensor to LDR Task.
xQueueHandle Queue_DMA_ProcessData;   		 //Send the read data from Microphone sensor to Process Data Task.
xQueueHandle Queue_SensorFusion_DataMining;//Send the read data from capacitive sensors, LDR sensor and microphone sensor in one only structure of data that was fused.
xQueueHandle Queue_DataMining_Make;				 //Send the pattern that we obtained with analysis the data to Make Graph Task.

/*********************************************************************************************************************************************************************/

#endif //_MAIN_H

