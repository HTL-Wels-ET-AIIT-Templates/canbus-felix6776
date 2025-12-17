/**
 ******************************************************************************
 * @file           : can.c
 * @brief          : CAN handling functions
 ******************************************************************************
 */
#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "stm32f429i_discovery_lcd.h"
#include "tempsensor.h"

/* Private define ------------------------------------------------------------*/
#define   CAN1_CLOCK_PRESCALER    16

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef     canHandle;

/* Private function prototypes -----------------------------------------------*/
static void initGpio(void);
static void initCanPeripheral(void);

/**
 * Initialize hardware GPIO and CAN peripheral
 */
void canInitHardware(void) {
	initGpio();
	initCanPeripheral();
}

/**
 * canInit function, set up hardware and display
 */
void canInit(void) {
	canInitHardware();

	LCD_SetFont(&Font12);
	LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);
	LCD_SetPrintPosition(3,1);
	printf("CAN1: Send-Recv");

	LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);
	LCD_SetPrintPosition(5,1);
	printf("Send-Cnt:");
	LCD_SetPrintPosition(5,15);
	printf("%5d", 0);
	LCD_SetPrintPosition(7,1);
	printf("Recv-Cnt:");
	LCD_SetPrintPosition(7,15);
	printf("%5d", 0);
	LCD_SetPrintPosition(9,1);
	printf("Send-Data:");
	LCD_SetPrintPosition(15,1);
	printf("Recv-Data:");

	LCD_SetPrintPosition(30,1);
	printf("Bit-Timing-Register: 0x%lx", CAN1->BTR);

	tempSensorInit();
}

/**
 * sends a CAN frame, if mailbox is free
 */
void canSendTask(void) {
	static unsigned int sendCnt = 0;
	float currentTemp = 0;
	uint32_t txMailbox;

	// Get temperature value
	currentTemp = tempSensorGetTemperature();

	// Convert to fixed point (e.g., 25.5 -> 255) for transmission
	uint16_t tempPayload = (uint16_t)(currentTemp * 10);

	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8] = {0};

	txHeader.StdId = 0x1AB;
	txHeader.ExtId = 0x00;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 3; // 1 byte constant + 2 bytes temperature

	txData[0] = 0xC3; // Status byte
	txData[1] = (uint8_t)(tempPayload & 0xFF);        // Low byte
	txData[2] = (uint8_t)((tempPayload >> 8) & 0xFF); // High byte

	// Request Transmission
	if (HAL_CAN_AddTxMessage(&canHandle, &txHeader, txData, &txMailbox) == HAL_OK) {
		sendCnt++;

		// Display send counter
		LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);
		LCD_SetPrintPosition(5,15);
		printf("%5u", sendCnt);

		// Display sent data (Hex)
		LCD_SetPrintPosition(11,1);
		printf("ID:0x1AB D:%02X %02X %02X", txData[0], txData[1], txData[2]);

        // Display temperature for clarity
        LCD_SetPrintPosition(12,1);
        printf("Sent T: %.2f C", currentTemp);
	}
}

/**
 * checks if a can frame has been received and shows content on display
 */
void canReceiveTask(void) {
	static unsigned int recvCnt = 0;
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	// Check if CAN frame has been received in FIFO 0
	if (HAL_CAN_GetRxFifoFillLevel(&canHandle, CAN_RX_FIFO0) > 0) {

		// Get CAN frame from RX fifo
		if (HAL_CAN_GetRxMessage(&canHandle, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
			recvCnt++;

			// Display recv counter
			LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);
			LCD_SetPrintPosition(7,15);
			printf("%5u", recvCnt);

			// Display received data
			LCD_SetPrintPosition(17,1);
			printf("ID:0x%03lX DLC:%lu", rxHeader.StdId, rxHeader.DLC);
			LCD_SetPrintPosition(18,1);
			printf("Data: %02X %02X %02X %02X", rxData[0], rxData[1], rxData[2], rxData[3]);
		}
	}
}

static void initGpio(void) {
	GPIO_InitTypeDef  canPins;
	__HAL_RCC_GPIOB_CLK_ENABLE();

	canPins.Alternate = GPIO_AF9_CAN1;
	canPins.Mode = GPIO_MODE_AF_OD;
	canPins.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	canPins.Pull = GPIO_PULLUP;
	canPins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &canPins);
}

static void initCanPeripheral(void) {
	CAN_FilterTypeDef canFilter;
	__HAL_RCC_CAN1_CLK_ENABLE();

	canHandle.Instance = CAN1;
	canHandle.Init.TimeTriggeredMode = DISABLE;
	canHandle.Init.AutoBusOff = DISABLE;
	canHandle.Init.AutoWakeUp = DISABLE;
	canHandle.Init.AutoRetransmission = ENABLE;
	canHandle.Init.ReceiveFifoLocked = DISABLE;
	canHandle.Init.TransmitFifoPriority = DISABLE;
	canHandle.Init.Mode = CAN_MODE_NORMAL;
	canHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
	canHandle.Init.TimeSeg1 = CAN_BS1_15TQ;
	canHandle.Init.TimeSeg2 = CAN_BS2_6TQ;
	canHandle.Init.Prescaler = CAN1_CLOCK_PRESCALER;

	if (HAL_CAN_Init(&canHandle) != HAL_OK) {
		Error_Handler();
	}

	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilter.FilterIdHigh = 0x0000;
	canFilter.FilterIdLow = 0x0000;
	canFilter.FilterMaskIdHigh = 0x0000;
	canFilter.FilterMaskIdLow = 0x0000;
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilter.FilterActivation = ENABLE;
	canFilter.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&canHandle, &canFilter) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&canHandle) != HAL_OK) {
		Error_Handler();
	}
}

void CAN1_RX0_IRQHandler(void) {
	HAL_CAN_IRQHandler(&canHandle);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Processing is done via polling in main loop as requested
}
