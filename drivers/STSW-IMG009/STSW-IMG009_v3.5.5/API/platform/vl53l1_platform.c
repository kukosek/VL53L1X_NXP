/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "board.h"
#include "peripherals.h"
#include "fsl_lpi2c.h"
#include "fsl_debug_console.h"
#include "vl53l1_error_codes.h"

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1


uint8_t _I2CBuffer[256];


uint8_t VL53L1_Raw7Bit(uint16_t Dev) {
	return (uint8_t)Dev >> 1;
}

int _I2CWrite(uint16_t Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    uint8_t dev_u8 = VL53L1_Raw7Bit(Dev); // shift right for raw 7bit addr
    LPI2C_MasterStart(LPI2C2, dev_u8, kLPI2C_Write);

    status = LPI2C_MasterSend(LPI2C2, pdata, count, false);
    if (status != kStatus_Success) {
    	PRINTF("I2C error on write");
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&§);
    	LPI2C_MasterInit(LPI2C2, &sMasterConfig);
    }

    LPI2C_MasterStop(LPI2C2);

    return status == kStatus_Success;
}

int _I2CRead(uint16_t Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    uint8_t dev_u8 = VL53L1_Raw7Bit(Dev); // shift right for raw 7bit addr
	LPI2C_MasterStart(LPI2C2, dev_u8, kLPI2C_Read);

    status = LPI2C_MasterReceive(LPI2C2, pdata, count, false);

	if (status != kStatus_Success) {
		PRINTF("I2C error on read");
		//VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
		//XNUCLEO6180XA1_I2C1_Init(&hi2c1);
	}

	LPI2C_MasterStop(LPI2C2);

	return status == kStatus_Success;
}

int8_t VL53L1_WriteMulti(uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    int8_t Status = VL53L1_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, count + 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
int8_t VL53L1_ReadMulti(uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int8_t Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, pdata, count);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_WrByte(uint16_t Dev, uint16_t index, uint8_t data) {
    int8_t Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data;

    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 3);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_WrWord(uint16_t Dev, uint16_t index, uint16_t data) {
    int8_t Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data >> 8;
    _I2CBuffer[3] = data & 0x00FF;

    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_WrDWord(uint16_t Dev, uint16_t index, uint32_t data) {
    int8_t Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = (data >> 24) & 0xFF;
    _I2CBuffer[3] = (data >> 16) & 0xFF;
    _I2CBuffer[4] = (data >> 8)  & 0xFF;
    _I2CBuffer[5] = (data >> 0 ) & 0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 6);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_UpdateByte(uint16_t Dev, uint16_t index, uint8_t AndData, uint8_t OrData) {
    int8_t Status = VL53L1_ERROR_NONE;
    uint8_t data;

    Status = VL53L1_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L1_WrByte(Dev, index, data);
done:
    return Status;
}

int8_t VL53L1_RdByte(uint16_t Dev, uint16_t index, uint8_t *data) {
    int8_t Status = VL53L1_ERROR_NONE;
    int32_t status_int;

	_I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, data, 1);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_RdWord(uint16_t Dev, uint16_t index, uint16_t *data) {
    int8_t Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);

    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
done:
    VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_RdDWord(uint16_t Dev, uint16_t index, uint32_t *data) {
    int8_t Status = VL53L1_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;
    VL53L1_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];

done:
    VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	return status;
}
