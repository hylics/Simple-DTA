/**
  ******************************************************************************
  * File Name          : freertos.c
  * Date               : 22/07/2015 15:33:36
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

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId adcTaskHandle;
osThreadId tuneTaskHandle;
osThreadId uartTaskHandle;
osMutexId Mutex_T_Handle;

/* USER CODE BEGIN Variables */
extern AD779X_HandleTypeDef adi1;
//extern SavedDomain_t EepromDomain;
extern SavedDomain_t Options_rw;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart4;
extern HAL_StatusTypeDef (*pf_output[N_FUNC_PWR])(float32_t pwr);
//extern HD44780_STM32F0xx_GPIO_Driver lcd_pindriver;
__IO static Temperature_t temp_handle = {0.0f};
arm_pid_instance_f32 pid_instance_1;
//__IO static float32_t out_tr;
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

  /* Create the mutex(es) */
  /* definition and creation of Mutex_T_ */
  osMutexDef(Mutex_T_);
  Mutex_T_Handle = osMutexCreate(osMutex(Mutex_T_));

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartAdcTask function */
void StartAdcTask(void const * argument)
{

  /* USER CODE BEGIN StartAdcTask */
	TickType_t LastWakeTime = xTaskGetTickCount();
	const TickType_t adc_delay = 2000; //milliseconds
	const uint32_t mutex_T_wait = 500; //milliseconds
	static uint32_t filt_conv_rtd;
	osDelay(1000); //for maxcount calibration
	//LastWakeTime = xTaskGetTickCount();
	
	uint8_t msg1[]="adc_task\n";
	
  /* Infinite loop */
  for(;;)
  {
		/*reading data unstable, some times its read only first byte of data*/
		__IO uint32_t raw_conv_rtd = 0;
		
		//la_adc_task = 1;
		HAL_UART_Transmit(&huart4, msg1, sizeof(msg1), 5);
		
		adi1.io &= ~AD779X_IEXCDIR(0x3);
		adi1.io |= AD779X_IEXCDIR(AD779X_DIR_IEXC1_IOUT2_IEXC2_IOUT1);
		
		taskENTER_CRITICAL();
		AD779X_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_conv_rtd = AD779X_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		
		adi1.io &= ~AD779X_IEXCDIR(0x3);
		adi1.io |= AD779X_IEXCDIR(AD779X_DIR_IEXC1_IOUT1_IEXC2_IOUT2);
		
		taskENTER_CRITICAL();
		AD779X_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_conv_rtd += AD779X_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		
		filt_conv_rtd = rec_filter(raw_conv_rtd, 55, 8); // 45=30s, 55=20s
		
		//access through semaphores to temperature state;
		osStatus status = osMutexWait(Mutex_T_Handle, mutex_T_wait);
		if(status == osOK) {
			temp_handle.rtd = rtd_get_temp(filt_conv_rtd, Pt375, r1000);
			temp_handle.setpoint = 25.0f;
			osMutexRelease(Mutex_T_Handle);
		}
		else {
			//do something when error occuring
		}
		
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
