## FreeRTOS Project

This repository for a group project from the Embedded Systems Course at the University of Science. In this course, we use the Open405R-C Package A, an STM32F4 development board. But a team just has only 1 board, So the decision is I will take the STM32F4 development board and other members will develope on their STM32F103C8T6. By using the HAL layers, this allows us to write portable code that can work across different STM32 devices with minimal changes.

The project's requirement is to configure the RTOS operating system on the board,. It has three different tasks.

**Task 1:** Read the voltage value from the ADC circuit and store it in the EEPROM on the chip through DMA.

**Task 2:** Continuously monitor any button press. If that button is pressed, send the value just read from Task 1 to CAN_1 channel.

**Task 3:** Continuously monitor CAN_2 channel, if there is data, display that data on LCD. (CAN_1 is interconnected with CAN_2).




### System Configuration

![image](https://github.com/user-attachments/assets/409ea06d-8320-4cbe-9e38-c9c95b9f45b3)

### Clock Configuration

![image](https://github.com/user-attachments/assets/a3f6c739-d17f-496e-952a-cd1e66fa5ba4)

### GPIO: Help in debugging the system and interacting with LCD

![image](https://github.com/user-attachments/assets/c5651ff5-fe5d-4394-a1fd-07208a49077c)


## Task 1: 

**Target:** Read the voltage value from the ADC circuit and store it in the EEPROM on the chip through DMA.

**DMA:** Read ADC and write to FLASH memory region.

![image](https://github.com/user-attachments/assets/566e6e41-0223-4971-a094-bf6d5a475979)

**NVIC**

![image](https://github.com/user-attachments/assets/c5df43ad-a798-487e-a2e9-dd23ea31aeec)

**TIM 4**

![image](https://github.com/user-attachments/assets/02f3dfba-603f-49e5-a6f9-d92ddae2589f)

**ADC**

![image](https://github.com/user-attachments/assets/aa54f3d5-b72a-4c9a-89ff-a8671b1a074a)

![image](https://github.com/user-attachments/assets/d6cf3ae6-fbe6-4f7d-b40e-0cf1dabf51af)

**Source Code**

In the `StartTask01()` function, we first start the TIM4 in interrupt mode.


```
rc = HAL_TIM_Base_Start_IT(&htim4);
assert(rc == HAL_OK && "Cannot start TIM4 clock for ADC sampling application");
```

Then initialize the necessary info to erase a sector when writing to FLASH

```
pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS; 
pEraseInit.Sector = FLASH_SECTOR_2; 
pEraseInit.NbSectors = 1; 
pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
```

Following that we get into the forever loop, here we always want this Task to sleep and only wake up to process data when there is an item in the Queue.

```
// event.value.v hold the actual message (eg. the ADC1 data)
osEvent event = osMessageGet(ADC_QNameHandle, osWaitForever);
```

If an item exist in the Queue, the Task wakeup and process the following line of code

```
// Unlock flash
HAL_FLASH_Unlock();
// Erase first
HAL_FLASHEx_Erase(&pEraseInit, &PageError);
// Write to flash
FLASH_DMA_Write(FLASH_SECTOR_0_TO_4_OFFSET(2), event.value.v);
// Lock flash
HAL_FLASH_Lock();

// Read Flash
FLASH_Data = *(__IO uint32_t *)(FLASH_SECTOR_0_TO_4_OFFSET(2));
```

But how does the Queue contain item such that it let Task 1 wakeup ? That's the jobs of TIM4 and DMA. The TIM4 will trigger an interrupt one every 500ms, which start the ADC and tell DMA controller to collect the ADC data once it finishes sampling. 

```
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM4) {
    HAL_ADC_Start_DMA(&hadc1, &ADC_Buffer, ADC_Buffer_Size);
  }
  /* USER CODE END Callback 1 */
}
```

When the DMA finish collecting the data, it will also trigger an interrupt so that we can collect the data and put it into the Queue - Task1 will wakeup after this interrupt finished.

```
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1) {
			osMessagePut(ADC_QNameHandle, ADC_Buffer, 50);
	}
}
```

## Task 03: 

**Task 3:** Continuously monitor CAN_2 channel, if there is data, display that data on LCD. (CAN_1 is interconnected with CAN_2).

(on progress) 

Currently, can display string on Touch LCD through SPI. I'm going to apply it into RTOS and read ADC data from Task1.

**SPI1**

![image](https://github.com/user-attachments/assets/7d49ab75-4faf-4cb2-8d8c-b3a5612976b6)



