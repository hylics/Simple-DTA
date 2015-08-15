/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"
#include "dbg_msg.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId adcTaskHandle;
osThreadId tuneTaskHandle;
osThreadId uartTaskHandle;
osMessageQId resultQueueHandle;

/* USER CODE BEGIN Variables */
extern AD779X_HandleTypeDef adi1;
//extern SavedDomain_t EepromDomain;
extern SavedDomain_t Options_rw;
//extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart4, huart1;
//extern HAL_StatusTypeDef (*pf_output[N_FUNC_PWR])(float32_t pwr);
//extern HD44780_STM32F0xx_GPIO_Driver lcd_pindriver;
//__IO static Temperature_t temp_handle = {0.0f};
//arm_pid_instance_f32 pid_instance_1;
//__IO static float32_t out_tr;
typedef struct __ADC_msg_t {
	uint32_t temp;
	uint32_t diff;
	uint8_t gain;
} ADC_msg_t;

size_t fre=0;
__IO static uint8_t CPU_IDLE = 0, malloc_err=0;


//unsigned int la_adc_task = 0;
//unsigned int la_pid_task = 0;
//fre=xPortGetFreeHeapSize();
//__IO static float32_t t_rtd = 0.0f;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartAdcTask(void const * argument);
void StartTuneTask(void const * argument);
void StartUartTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern void init_lcd(void);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	 static portTickType LastTick; 
        static uint32_t count;             //наш трудяга счетчик
        static uint32_t max_count ;                //максимальное значение счетчика, вычисляется при калибровке и соответствует 100% CPU idle

        count++;                                                  //приращение счетчика

        if((xTaskGetTickCount() - LastTick ) > 1000)    { //если прошло 1000 тиков (1 сек для моей платфрмы)
                LastTick = xTaskGetTickCount();
                if(count > max_count) max_count = count;          //это калибровка
                CPU_IDLE = (100 * count) / max_count;               //вычисляем текущую загрузку
                count = 0;                                        //обнуляем счетчик
        }

}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
__IO	static int8_t taskname[30];
	for(uint32_t i=0; i<30; i++) {
		taskname[i]=*(pcTaskName+i);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	malloc_err++;
}
/* USER CODE END 5 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of adcTask */
  osThreadDef(adcTask, StartAdcTask, osPriorityHigh, 0, 128);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

  /* definition and creation of tuneTask */
  osThreadDef(tuneTask, StartTuneTask, osPriorityBelowNormal, 0, 128);
  tuneTaskHandle = osThreadCreate(osThread(tuneTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of resultQueue */
  osMessageQDef(resultQueue, 8, ADC_msg_t);
  resultQueueHandle = osMessageCreate(osMessageQ(resultQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartAdcTask function */
void StartAdcTask(void const * argument)
{

  /* USER CODE BEGIN StartAdcTask */
	TickType_t LastWakeTime = xTaskGetTickCount();
	BaseType_t xStatus = pdFALSE;
	const TickType_t adc_delay = 2000; //milliseconds
	
	osDelay(1000); //for maxcount calibration
	
  /* Infinite loop */
  for(;;)
  {
		/**/
		uint32_t raw_temp = 0, raw_diff = 0, gain = AD779X_GAIN_64;
		ADC_msg_t tMsg;
		
		#ifndef NDEBUG
		//la_adc_task = 1;
		HAL_UART_Transmit(&huart4, (uint8_t *)msg1, sizeof(msg1), 5);
		#endif
		
		/*Read reference temperature at channel 1**********************************/
		adi1.conf &= ~(AD779X_CONF_CHAN(0xF) | AD779X_CONF_GAIN(0xF));
		adi1.conf |= AD779X_CONF_CHAN(AD779X_CH_AIN1P_AIN1M);
		adi1.conf |= ~AD779X_CONF_GAIN(AD779X_GAIN_1);
		adi1.io &= ~AD779X_IEXCDIR(0x3);
		adi1.io |= AD779X_IEXCDIR(AD779X_DIR_IEXC1_IOUT2_IEXC2_IOUT1);
		
		taskENTER_CRITICAL();
		AD779X_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_temp = AD779X_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		
		adi1.io &= ~AD779X_IEXCDIR(0x3);
		adi1.io |= AD779X_IEXCDIR(AD779X_DIR_IEXC1_IOUT1_IEXC2_IOUT2);
		
		taskENTER_CRITICAL();
		AD779X_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_temp += AD779X_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		/***************************************************************************/
		
		
		/*Read difference temperature at channel 2**********************************/
		adi1.conf &= ~(AD779X_CONF_CHAN(0xF) | AD779X_CONF_GAIN(0xF));
		adi1.conf |= AD779X_CONF_CHAN(AD779X_CH_AIN2P_AIN2M);
		adi1.conf |= ~AD779X_CONF_GAIN(gain);
		adi1.io &= ~AD779X_IEXCDIR(0x3);
		adi1.io |= AD779X_IEXCDIR(AD779X_DIR_IEXC1_IOUT2_IEXC2_IOUT1);
		
		taskENTER_CRITICAL();
		AD779X_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_diff = AD779X_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		
		adi1.io &= ~AD779X_IEXCDIR(0x3);
		adi1.io |= AD779X_IEXCDIR(AD779X_DIR_IEXC1_IOUT1_IEXC2_IOUT2);
		
		taskENTER_CRITICAL();
		AD779X_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_diff += AD779X_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		/***************************************************************************/
		
		/*Send message**************************************************************/
		tMsg.temp = raw_temp;
		tMsg.diff = raw_diff;
		tMsg.gain = (uint8_t)gain;
		
		if (0 != resultQueueHandle) {
			xStatus = xQueueSend( resultQueueHandle, (void *)&tMsg, (TickType_t)100 );
		}
		#ifndef NDEBUG
		else {
			//if queue not created send message
			HAL_UART_Transmit(&huart4, (uint8_t *)err1, sizeof(err1), 5);
		}
		if(pdTRUE != xStatus) {
			//if queue full
			HAL_UART_Transmit(&huart4, (uint8_t *)err2, sizeof(err2), 5);
		}
		#endif
		
		/***************************************************************************/
		
		
		/*Tune gain*****************************************************************/
		if (DIFF_SCALE_MAX < raw_diff) {
			if(AD779X_GAIN_1 < gain) {
				gain--;
			}
		}
		if (DIFF_SCALE_MIN > raw_diff) {
			if (AD779X_GAIN_128 > gain) {
				gain++;
			}
		}
		/***************************************************************************/
		
		fre=xPortGetFreeHeapSize();
		
		//la_adc_task = 0;
		
		//osDelay(2000);
		vTaskDelayUntil(&LastWakeTime, adc_delay); //1000 ms
  }
	//vTaskDelete(NULL);
  /* USER CODE END StartAdcTask */
}

/* StartTuneTask function */
void StartTuneTask(void const * argument)
{
  /* USER CODE BEGIN StartTuneTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTuneTask */
}

/* StartUartTask function */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTask */
	ADC_msg_t rMsg;
	float32_t temp, dta;
	BaseType_t xStatus = pdFALSE;
	
  /* Infinite loop */
  for(;;)
  {
		#ifndef NDEBUG
		HAL_UART_Transmit(&huart4, (uint8_t *)msg2, sizeof(msg2), 5);
		#endif
		
		/*Receive message***********************************************************/
		if (0 != resultQueueHandle) {
			xStatus = xQueueReceive(resultQueueHandle, &rMsg, (TickType_t)10000);
		}
		#ifndef NDEBUG
		else {
			//if queue not created send message
			HAL_UART_Transmit(&huart4, (uint8_t *)err1, sizeof(err1), 5);
		}
		
		if(pdTRUE != xStatus) {
			//if data not received
			HAL_UART_Transmit(&huart4, (uint8_t *)err2, sizeof(err2), 5);
		}
		#endif
		/***************************************************************************/
		
		/*calculate*****************************************************************/
		temp = rtd_get_temp(rMsg.temp, Pt375, r1000);
		dta = rMsg.diff / rMsg.gain;
		/***************************************************************************/
		
		/*format string and send message to PC**************************************/
		//HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
		
		/***************************************************************************/
    osDelay(1);
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
