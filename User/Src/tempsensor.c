/**
 * Simplify usage of DS18B20 temperature sensor
 *
 */


#include "tempsensor.h"
#include "DS18B20.h"
#include "main.h"

// function declarations

/**
 * Initialize peripherals for DS18B20
 * return 1 if a device has been found
 */
void tempSensorInit(void)
{
	GPIO_InitTypeDef pg9;

	__HAL_RCC_GPIOG_CLK_ENABLE();
	pg9.Pin = GPIO_PIN_9;
	pg9.Mode = GPIO_MODE_OUTPUT_PP;
	pg9.Pull = GPIO_PULLUP;
	pg9.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	pg9.Alternate = 0;
	HAL_GPIO_Init(GPIOG, &pg9);

	ds1820_init(GPIOG, GPIO_PIN_9);
}

/**
 * Get
 * return 1 if a device has been found
 */
float tempSensorGetTemperature() // in °C
{
	float temperature = -1e3;

	temperature = ds1820_read_temp(GPIOG, GPIO_PIN_9);
	return temperature;
}
