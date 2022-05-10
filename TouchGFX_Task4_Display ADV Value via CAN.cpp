
/*
04.05.2022
upload the initial GUI setting, without programming.

check the FMC-SDRAM1-Clock and chip enable(SDCKE0+...)+Internal bank number(4 banks)+ Address(12 bits)+Data(32 bits)+ Byte enable(32-bit byte enable)


PB9-Can1 TX
PB8-Can1 RX
CAN1 RX0-enabled
TIM10 - Prescaler(PSC-16 bits value):18000 - Counter Period:1875

*/





STM32F446RE
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint16_t TxData[8];
uint16_t RxData[8];
uint16_t readValue;

uint32_t TxMailbox;
uint16_t timer_val; //timer value used to hold our timer value and elapsed time calculations
/* USER CODE END 0 */


/* USER CODE BEGIN 2 */


// Start timer
HAL_TIM_Base_Start(&htim10);
//Get current time
timer_val = __HAL_TIM_GET_COUNTER(&htim10);

// Start CAN
HAL_CAN_Start(&hcan1);

// Activate the notification
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

TxHeader.DLC = 2;  // data length
TxHeader.IDE = CAN_ID_STD;
TxHeader.RTR = CAN_RTR_DATA;
TxHeader.StdId = 0x446;  // ID

/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
  while (1)
    {
	//here
	//Start ADC conversion and read Potentiometer value
	 HAL_ADC_Start(&hadc1);

	//causes the processor to hang while it waits for an ADC conversion to complete
	 HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);

		if (__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 125)
		{
			//Blink the LED on the first board
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			timer_val = __HAL_TIM_GET_COUNTER(&htim10);

			//once the ADC conversion and the timer is done,
			//we get the value from the ADC channel register and store the raw value
			TxData[0] = HAL_ADC_GetValue(&hadc1);
			TxData[1] = 300;    // update time the on the GUI

			//Transfer the Potentiometer data by CAN-Bus
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			//HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
			//HAL_Delay(100);
        }		
    }
/* USER CODE END WHILE */





//---------------------------------------------------------------------------

STM32F769_Discovery Kit


main.c
#include "stm32f7xx_hal_can.h" //include the can function and header


//STEP1---------------------------------------------------------------------------

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t Angle = 0; //create a variable to keep the track of this angle
uint16_t Get_Value = 0;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint16_t TxData[8];
uint16_t RxData[8];

uint32_t TxMailbox;
int datacheck = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.DLC == 2)
	{
		datacheck = 1;
	}
}

/* USER CODE END 0 */


//STEP2---------------------------------------------------------------------------


  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);

  // Activate the notification
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeader.DLC = 2;  // data length
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x103;  // ID

  /* USER CODE END 2 */



  //STEP3---------------------------------------------------------------------------
/* USER CODE END Header_StartadcTask */
void StartadcTask(void *argument)
{
  /* USER CODE BEGIN StartadcTask */

	uint16_t ADC_VAL; //define a variable to store the adc value
	//uint16_t Angle = 0 ;
	//uint16_t Get_Value = 0 ;
  /* Infinite loop */
  for(;;) //findme
  {
	  if (datacheck)
	  {
		  Get_Value = RxData[0];
		  Angle = RxData[0]*300/4095 ;
		  osDelay(RxData[1]);
	  }
	  //function to read the ADC value and convert it to angle
	  //HAL_ADC_Start (&hadc1); // Start ADC
	  //HAL_ADC_PollForConversion (&hadc1, 100);
	  //HDC_VAL = HAL_ADC_GetValue (&hadc1);
	  //HAL_ADC_Stop (&hadc1);

	  //Angle = ADC_VAL;
	  //Get_Value = ADC_VAL ;
	  //Angle = ADC_VAL*300/4095 ;

    //osDelay(100);
  }
  /* USER CODE END StartadcTask */
}

or

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for(;;)
  {
	//here
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	//HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
	Get_Value = RxData[0];
	Angle = RxData[0]*300/4095 ;

	osDelay(RxData[1]);
	//HAL_GPIO_TogglePin (GPIOJ, GPIO_PIN_5);
	//osDelay(500);

    //osDelay(100);
  }
  /* USER CODE END 5 */
}

  //STEP4---------------------------------------------------------------------------
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig; //canfilter

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x446<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x446<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */





//--------------------------------------------------------------------------
Screen1View.cpp
extern uint16_t Angle;
extern uint16_t Get_Value;

void Screen1View::handleTickEvent()
{
	circleProgress1.setValue(Angle);
	boxProgress1.setValue(Angle);
	textProgress1.setValue(Angle);
	Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%u", Get_Value);
	textArea1.invalidate();
}


Screen1View.hpp
class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void handleTickEvent(); //this function is basically called every frame, so its rate depends on display

protected:

};

#endif // SCREEN1VIEW_HPP