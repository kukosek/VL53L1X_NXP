/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include <stdio.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    char ch;

    /* Init board hardware. */
    BOARD_InitHardware();

    PRINTF("hello world.\r\n");
    int i = 0;

    while (1)
    {
        ch = GETCHAR();
        PUTCHAR(ch);

        char i_str[20];
        sprintf(i_str, "%d\r\n", i);

        PRINTF(i_str);
        i++;
    }
}
