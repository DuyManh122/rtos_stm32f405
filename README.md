## FreeRTOS Project

This repository for a group project from the Embedded Systems Course at the University of Science. In this course, we use the Open405R-C Package A, an STM32F4 development board. But a team just has only 1 board, So the decision is I will take the STM32F4 development board and other members will develope on their STM32F103C8T6. By using the HAL layers, this allows us to write portable code that can work across different STM32 devices with minimal changes.

The project's requirement is to configure the RTOS operating system on the board,. It has three different tasks.

**Task 1:** Read the voltage value from the ADC circuit and store it in the EEPROM on the chip through DMA.

**Task 2:** Continuously monitor any button press. If that button is pressed, send the value just read from Task 1 to CAN_1 channel.

**Task 3:** Continuously monitor CAN_2 channel, if there is data, display that data on LCD. (CAN_1 is interconnected with CAN_2).




### System Configuration

![image](https://github.com/user-attachments/assets/a3a5b0e3-d123-44e6-99c1-549254331486)

### Clock Configuration

![image](https://github.com/user-attachments/assets/d31380bd-ead7-499c-8fcc-217aac18f665)

### GPIO: Help in debugging the system and interacting with LCD

![image](https://github.com/user-attachments/assets/edd0b0bd-20b7-48c8-896c-20459332911a)

## Task 1: 

**Target:** Read the voltage value from the ADC circuit and store it in the EEPROM on the chip through DMA.

**Task 01 Configuration**

![image](https://github.com/user-attachments/assets/01dc30bc-8b77-4719-bd70-cbdeac134bec)

**DMA:** Read ADC and write to FLASH memory region.

![image](https://github.com/user-attachments/assets/566e6e41-0223-4971-a094-bf6d5a475979)

**NVIC**

![image](https://github.com/user-attachments/assets/c5df43ad-a798-487e-a2e9-dd23ea31aeec)

**TIM 4**

![image](https://github.com/user-attachments/assets/f385f96f-0e8a-40f5-a5bd-2910f624fa7e)

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
## Task 2: 

**Target:** Continuously monitor one button. When this button is pressed, sends the value read from Task01 to CAN1.

**Task02 Configuration:**

![image](https://github.com/user-attachments/assets/744be91f-96ea-44b8-a753-a4a548cd6f79)

**CAN1 Parameter Settings:** 

![image](https://github.com/user-attachments/assets/e6826098-f179-4b41-b5f5-b391060ea59a)

![image](https://github.com/user-attachments/assets/2670537d-68bb-4c7e-99cc-3343836c50e1)

**CAN1 GPIO Settings:**

![image](https://github.com/user-attachments/assets/4e4b8080-ed36-4e18-ac5f-6bc1d9263763)

**Source Code**

In the `main()` function, we first start the CAN1.

```
/*Start Can*/
HAL_CAN_Start(&hcan1);
```

CAN1 transmission header configuration 
```
TxHeader.DLC = 2; 				// DataLength
TxHeader.IDE = CAN_ID_STD; 			// Can Identifier type
TxHeader.StdId = 0x405;				// Standand ID
TxHeader.RTR = CAN_RTR_DATA;			// CAN Remote Transmission Request
TxHeader.TransmitGlobalTime = DISABLE;
```

CAN1 filter configuration. After this configuration CAN1 will accept all IDs in CAN Bus. The remaining filters from 12 to 27 will be assigned to CAN2 (slave CAN).

```
CAN_FilterTypeDef canfilterconfig;


canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
canfilterconfig.FilterBank = 0;  // which filter bank to use from the assigned ones
canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
canfilterconfig.FilterIdHigh = 0x0000;
canfilterconfig.FilterIdLow = 0x0000;
canfilterconfig.FilterMaskIdHigh = 0x0000;
canfilterconfig.FilterMaskIdLow = 0x0000;
canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
canfilterconfig.SlaveStartFilterBank = 12;  // how many filters to assign to the CAN1 (master can)

HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
```

In the `StartTask02()` function, we first wait for binary semaphore `myBinarySem01Handle` to become available.

`
osSemaphoreWait(myBinarySem01Handle, osWaitForever);
`

The binary semaphore will available when user key is pressed. This will trigger `HAL_GPIO_EXTI_Callback()` function.

```
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  currentMillis = HAL_GetTick();
  if (GPIO_Pin == GPIO_PIN_1 && (currentMillis - previousMillis > 100))
  {
		previousMillis = currentMillis;
		
		//Release Semaphore
		osSemaphoreRelease(myBinarySem01Handle);
  }
}
```

When binary semaphore becomes available, we will transmit data through CAN1

```
//Get Data From Flash in task01
TxData[0] = FLASH_Data & 0xFF;  
TxData[1] = (FLASH_Data >> 8) & 0xFF;
datacheck = 1;
if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
{
		Error_Handler ();
}
```

## Task 03: 

**Task 3:** Continuously monitor CAN_2 channel, if there is data, display that data on LCD. (CAN_1 is connected with CAN_2).

**Task03 Configuration**

![image](https://github.com/user-attachments/assets/495899fb-5b93-407e-8d4a-cc924e5ecbda)

**SPI1 Parameter Settings**

![image](https://github.com/user-attachments/assets/ef73fea1-e1f0-4dbe-ad62-1999bf363732)

**SPI1 GPIO Settings**

![image](https://github.com/user-attachments/assets/2bd6e3af-6105-44a1-98ca-9ccc3f49f652)

**CAN2 Parameter Setting**

![image](https://github.com/user-attachments/assets/f8ef6615-c44d-4be9-9530-a9cbe252c079)
![image](https://github.com/user-attachments/assets/3239d190-9f90-45a4-b8a4-3e2a1d5adcb4)

**CAN2 NVIC Setting**

![image](https://github.com/user-attachments/assets/de91a5e3-bfda-4b6f-814f-39b1d8c0e51e)

**CAN2 GPIO Setting**

![image](https://github.com/user-attachments/assets/8340e347-5a32-471b-b6c4-18efbfdb23b8)

**Source Code**

In the `main()` function, we first start the CAN2 and enable RX FIFO0 message pending interrupt. This interrupt will trigger whenever a new message arrives in FIFO0 of the CAN2 peripheral

```

HAL_CAN_Start(&hcan2);

/* Enable Can Interrupt */
HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
```

After that, we will initialize ST7789 touch LCD display

```
ST7789_Init();
```

The `ST7789_Init()` function will initialize LCD Pins, reset Hardware, send some basic initialization commands and turn on the displays

```
HAL_Delay(10);
ST7789_RST_Set();
ST7789_Select();
ST7789_BLK_Set();
ST7789_RST_Set();
HAL_Delay(5);
ST7789_RST_Clr();
HAL_Delay(5);
ST7789_RST_Set();
HAL_Delay(5);

ST7789_WriteCommand(ST7789_SLPOUT);	//	Out of sleep mode
HAL_Delay(10);
ST7789_WriteCommand(ST7789_MADCTL);
ST7789_WriteSmallData(0x00);		
ST7789_WriteCommand(ST7789_COLMOD);		//	Set color mode
ST7789_WriteSmallData(0x05);
ST7789_WriteCommand(0xB2);				//	Porch control
{
uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
ST7789_WriteData(data, sizeof(data));
}
ST7789_SetRotation(ST7789_ROTATION);	//	MADCTL (Display Rotation)

/* Internal LCD Voltage generator settings */
ST7789_WriteCommand(0XB7);				//	Gate Control
ST7789_WriteSmallData(0x35);			//	Default value
ST7789_WriteCommand(0xBB);				//	VCOM setting
ST7789_WriteSmallData(0x28);			//	0.725v (default 0.75v for 0x20)
ST7789_WriteCommand(0xC0);				//	LCMCTRL	
ST7789_WriteSmallData (0x3C);			//	Default value
ST7789_WriteCommand (0xC2);				//	VDV and VRH command Enable
ST7789_WriteSmallData (0x01);			//	Default value
ST7789_WriteCommand (0xC3);				//	VRH set
ST7789_WriteSmallData (0x0b);			//	+-4.45v (defalut +-4.1v for 0x0B)
ST7789_WriteCommand (0xC4);				//	VDV set
ST7789_WriteSmallData (0x20);			//	Default value
ST7789_WriteCommand (0xC6);				//	Frame rate control in normal mode
ST7789_WriteSmallData (0x0F);			//	Default value (60HZ)
ST7789_WriteCommand (0xD0);				//	Power control
ST7789_WriteSmallData (0xA4);			//	Default value
ST7789_WriteSmallData (0xA1);			//	Default value
/**************** Division line ****************/

ST7789_WriteCommand(0xE0);
{
uint8_t data[] = {0xD0, 0x01, 0x08, 0x0f, 0x11, 0x2a, 0x36, 0x55, 0x44, 0x3a, 0x0b, 0x06, 0x11, 0x20};
ST7789_WriteData(data, sizeof(data));
}

ST7789_WriteCommand(0xE1);
{
uint8_t data[] = {0xD0, 0x02, 0x07, 0x0a, 0x0b, 0x18, 0x34, 0x43, 0x4a, 0x2b, 0x1b, 0x1c, 0x22, 0x1f};
ST7789_WriteData(data, sizeof(data));
}
//    ST7789_WriteCommand (ST7789_INVON);		//	Inversion ON
//  	ST7789_WriteCommand (ST7789_NORON);		//	Normal Display on
ST7789_WriteCommand (ST7789_DISPON);	//	Main screen turned on	

HAL_Delay(50);
ST7789_Fill_Color(WHITE);				//	Fill with White.
```

CAN2 filter configuration. After this configuration CAN2 will accept all IDs in CAN Bus.

```
CAN_FilterTypeDef canfilterconfig;	

canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
canfilterconfig.FilterBank = 14;  // which filter bank to use from the assigned ones
canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
canfilterconfig.FilterIdHigh = 0x0000;
canfilterconfig.FilterIdLow = 0x0000;
canfilterconfig.FilterMaskIdHigh = 0x0000;
canfilterconfig.FilterMaskIdLow = 0x0000;
canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);	
```

The `HAL_CAN_RxFifo0MsgPendingCallback()` function is triggered by an interrupt when a new message arrives in the CAN2_RX_FIFO_0.

This function will receive the message from can1, store it to FreeRTOS queue and let Binary Semaphore to become available 

```
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
//debugging
datacheck = 2;
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

//Get Message from can 2
HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

//assign to LCD_Data variable
LCD_Data = (RxData[1] << 8) + RxData[0];

osMessagePut(LCD_QNameHandle, LCD_Data, 50);

//Release Semaphore
osSemaphoreRelease(myBinarySem02Handle);
}
```

The `StartTask03()` function will wait semaphore to become available. When the semaphore available, it will get the message from queue and write data to LCD
```
void StartTask03(void const * argument)
{
/* USER CODE BEGIN StartTask03 */
/* Infinite loop */
for(;;)
{

	//Wait binary semaphore02 release
	osSemaphoreWait(myBinarySem02Handle, osWaitForever);

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	
	//This variable just use for debugging
	datacheck = 1;

	osEvent event = osMessageGet(LCD_QNameHandle, osWaitForever);
	

	char S[20];
	sprintf(S,"ADC value is: %d / 4095", event.value.v);
	
	
	ST7789_Fill_Color(WHITE);
	ST7789_WriteString(40,150, S, Font_7x10, WHITE, BLACK);
}
/* USER CODE END StartTask03 */
}
```
