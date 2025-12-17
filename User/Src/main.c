/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "main.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include "ts_calibration.h"
#include "can.h"
#include "cancpp.h"

/* Private function prototypes -----------------------------------------------*/
static int GetUserButtonPressed(void);
static int GetTouchState (int *xCoord, int *yCoord);

void SysTick_Handler(void) {
	HAL_IncTick();
}

int main(void) {
	HAL_Init();
	SystemClock_Config();

	LCD_Init();
	TS_Init(LCD_GetXSize(), LCD_GetYSize());

	LCD_Clear(LCD_COLOR_BLACK);
	LCD_SetTextColor(LCD_COLOR_YELLOW);
	LCD_SetBackColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font20);
	LCD_SetPrintPosition(0, 0);
	printf("HTL Wels");

	LCD_SetFont(&Font8);
	LCD_SetColors(LCD_COLOR_MAGENTA, LCD_COLOR_BLACK);
	LCD_DisplayStringAtLineMode(39, "copyright CAN Experts!", CENTER_MODE);

	// Initialize CAN peripherals
	canInit();

	bool GetUserButtonPressedOld = false;

	while (1) {
		HAL_Delay(10); // Loop timing roughly 100Hz

		// Send data over CAN when user button is pressed (Rising Edge)
		bool buttonState = (bool)GetUserButtonPressed();
		if(buttonState && !GetUserButtonPressedOld) {
			canSendTask();
		}
		GetUserButtonPressedOld = buttonState;

		// Check if data has been received
		canReceiveTask();

		// Display Timer
		int cnt = HAL_GetTick();
		LCD_SetFont(&Font12);
		LCD_SetTextColor(LCD_COLOR_RED);
		LCD_SetPrintPosition(0, 18);
		printf("   Timer: %.1f", cnt/1000.0);

		// Touch Interface logic
		int x, y;
		if (GetTouchState(&x, &y)) {
			LCD_FillCircle(x, y, 5);
		}
	}
}

static int GetUserButtonPressed(void) {
	return (GPIOA->IDR & GPIO_PIN_0) ? 1 : 0; // PA0 check
}

static int GetTouchState (int* xCoord, int* yCoord) {
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
