/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * CAN Network
 * CAN1 is used, GPIO config: TX = PB9, RX = PB8
 * (Those GPIOs are also used for display, that's why the display is flickering when
 * CAN is used)
 *
 * Temperature sensor DS18B20 has to be connected to 3V3, GND and PG9
 *
 * Exercises are marked with ToDo. There are 2 stages of ToDos:
 * 1) Implement CAN communication with a counter value
 * 2) Add temperature value to CAN payload. ( Marked with ToDo (2) )
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "main.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include "ts_calibration.h"
#include "can.h"
#include "cancpp.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static int GetUserButtonPressed(void);
static int GetTouchState (int *xCoord, int *yCoord);

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	HAL_IncTick();
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/*  user variable declarations                                              */
	static bool flm = 0;
	static bool userButtonIsPressed = 0;





	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize LCD and touch screen */
	LCD_Init();
	TS_Init(LCD_GetXSize(), LCD_GetYSize());
	/* touch screen calibration */
	//	TS_Calibration();

	/* Clear the LCD and display basic starter text */
	LCD_Clear(LCD_COLOR_BLACK);
	LCD_SetTextColor(LCD_COLOR_YELLOW);
	LCD_SetBackColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font20);
	LCD_SetPrintPosition(0, 0);
	printf("HTL Wels");

	LCD_SetFont(&Font8);
	LCD_SetColors(LCD_COLOR_MAGENTA, LCD_COLOR_BLACK); // TextColor, BackColor
	LCD_DisplayStringAtLineMode(39, "copyright Haslinger", CENTER_MODE);

	// ToDo: set up CAN peripherals
	canInit();
	canInitHardware();


	/* Infinite loop */
	while (1)
	{
		//execute main loop every 100ms
		HAL_Delay(10);

		// ToDo: send data over CAN when user button has been pressed
		static uint8_t counter = 0;
		uint8_t txData[4];

		// Button pressed?
		userButtonIsPressed = GetUserButtonPressed();
		if (userButtonIsPressed && !flm) {
			canSendTask();
		    HAL_Delay(10); // debounce
		}
		flm = userButtonIsPressed;



		// ToDo: check if data has been received


		// display timer
		int cnt = HAL_GetTick();
		LCD_SetFont(&Font12);
		LCD_SetTextColor(LCD_COLOR_RED);
		LCD_SetPrintPosition(0, 18);
		printf("   Timer: %.1f", cnt/1000.0);

		// test touch interface
		int x, y;
		if (GetTouchState(&x, &y)) {
			LCD_FillCircle(x, y, 5);
		}


	}
}

/**
 * Check if User Button has been pressed
 * @param none
 * @return 1 if user button input (PA0) is high
 */
static int GetUserButtonPressed(void) {
	return (GPIOA->IDR & 0x0001);
}

/**
 * Check if touch interface has been used
 * @param xCoord x coordinate of touch event in pixels
 * @param yCoord y coordinate of touch event in pixels
 * @return 1 if touch event has been detected
 */
static int GetTouchState (int* xCoord, int* yCoord) {
	void    BSP_TS_GetState(TS_StateTypeDef *TsState);
	TS_StateTypeDef TsState;
	int touchclick = 0;

	TS_GetState(&TsState);
	if (TsState.TouchDetected) {
		*xCoord = TsState.X;
		*yCoord = TsState.Y;
		touchclick = 1;
		if (TS_IsCalibrationDone()) {
			*xCoord = TS_Calibration_GetX(*xCoord);
			*yCoord = TS_Calibration_GetY(*yCoord);
		}
	}

	return touchclick;
}


