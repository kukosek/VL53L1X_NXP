/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "board.h"
#include "app.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "peripherals.h"
#include "clock_config.h"
#include <stdio.h>


uint16_t dev=0x52;
int status=0;
volatile int IntCount;


int main(void)
{
    char ch;
    uint8_t byteData;
    uint16_t wordData;


    uint8_t sensorState=0;
    uint16_t Distance;
    uint16_t SignalRate;
    uint16_t AmbientRate;
    uint16_t SpadNum;
    uint8_t RangeStatus;
    uint8_t dataReady =0;
    uint16_t timeout_counter = 0;

    /* Init board hardware. */
    BOARD_InitHardware();
    BOARD_InitBootClocks();

    PRINTF("Hello world\r\n");

    // LPI2C_MasterStart(LPI2C2, dev, kLPI2C_Write);

    status = VL53L1_RdByte(dev, 0x010F, &byteData);
	PRINTF("VL53L1X Model_ID: %X\r\n", byteData);
	status = VL53L1_RdByte(dev, 0x0110, &byteData);
	PRINTF("VL53L1X Module_Type: %X\r\n", byteData);
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	PRINTF("VL53L1X: %X\r\n", wordData);

	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SDK_DelayAtLeastUs(100000, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	}
	PRINTF("Chip booted\r\n");

	/* This function must to be called to initialize the sensor with the default setting  */
	status = VL53L1X_SensorInit(dev);
	/* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	status = VL53L1X_SetInterMeasurementInMs(dev, 100); /* in ms, IM must be > = TB */
	status = VL53L1X_SetInterruptPolarity(dev,0); //This function programs the interrupt polarity, 1 = active high (default), 0 = active low.

	PRINTF("VL53L1X Ultra Lite Driver Example running ...\r\n");


	status = VL53L1X_StartRanging(dev);

	while (1) {
		// SW polling mode
		while (dataReady == 0) {
			status = VL53L1X_CheckForDataReady(dev, &dataReady);
			timeout_counter++;
			if (timeout_counter >= 1000)
			{
				status = (uint8_t)VL53L1X_ERROR_TIMEOUT;
				PRINTF("No data ready for long time, please check your system\n");
				timeout_counter = 0;
			}
			status = VL53L1_WaitMs(dev, 1);
		}
		dataReady = 0;
		timeout_counter = 0;

		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
		status = VL53L1X_GetDistance(dev, &Distance);
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);
		status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		PRINTF("%u, %u, %u, %u, %u\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
	}
}
