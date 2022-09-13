/**
 ******************************************************************************
 * @file    lsm303agr.c
 * @author  MEMS Software Solutions Team
 * @brief   LSM303AGR driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "lsm303agr.h"
#include "ros/lsm303agr_rosWrapper.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @defgroup LSM303AGR LSM303AGR
 * @{
 */

/** @defgroup LSM303AGR_Exported_Variables LSM303AGR Exported Variables
 * @{
 */

LSM303AGR_ACC_CommonDrv_t LSM303AGR_ACC_COMMON_Driver =
{
  LSM303AGR_ACC_Init,
  LSM303AGR_ACC_DeInit,
  LSM303AGR_ACC_ReadID,
  LSM303AGR_ACC_GetCapabilities,
};

LSM303AGR_ACC_Drv_t LSM303AGR_ACC_Driver =
{
  LSM303AGR_ACC_Enable,
  LSM303AGR_ACC_Disable,
  LSM303AGR_ACC_GetSensitivity,
  LSM303AGR_ACC_GetOutputDataRate,
  LSM303AGR_ACC_SetOutputDataRate,
  LSM303AGR_ACC_GetFullScale,
  LSM303AGR_ACC_SetFullScale,
  LSM303AGR_ACC_GetAxes,
  LSM303AGR_ACC_GetAxesRaw,
};

LSM303AGR_MAG_CommonDrv_t LSM303AGR_MAG_COMMON_Driver =
{
  LSM303AGR_MAG_Init,
  LSM303AGR_MAG_DeInit,
  LSM303AGR_MAG_ReadID,
  LSM303AGR_MAG_GetCapabilities,
};

LSM303AGR_MAG_Drv_t LSM303AGR_MAG_Driver =
{
  LSM303AGR_MAG_Enable,
  LSM303AGR_MAG_Disable,
  LSM303AGR_MAG_GetSensitivity,
  LSM303AGR_MAG_GetOutputDataRate,
  LSM303AGR_MAG_SetOutputDataRate,
  LSM303AGR_MAG_GetFullScale,
  LSM303AGR_MAG_SetFullScale,
  LSM303AGR_MAG_GetAxes,
  LSM303AGR_MAG_GetAxesRaw,
};

/**
 * @}
 */

/** @defgroup LSM303AGR_Private_Function_Prototypes LSM303AGR Private Function Prototypes
 * @{
 */

static int32_t ReadAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t ReadMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LSM303AGR_ACC_GetSensitivityHR(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity);
static int32_t LSM303AGR_ACC_GetSensitivityNM(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity);
static int32_t LSM303AGR_ACC_GetSensitivityLP(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity);
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Enabled(LSM303AGR_ACC_Object_t *pObj, float Odr);
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Disabled(LSM303AGR_ACC_Object_t *pObj, float Odr);

/**
 * @}
 */

/** @defgroup LSM303AGR_Exported_Functions LSM303AGR Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_RegisterBusIO(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_IO_t *pIO)
{
  int32_t ret = LSM303AGR_OK;

  if (pObj == NULL)
  {
    ret = LSM303AGR_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadAccRegWrap;
    pObj->Ctx.write_reg = WriteAccRegWrap;
    pObj->Ctx.handle    = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = LSM303AGR_ERROR;
    }
    else if (pObj->IO.Init() != LSM303AGR_OK)
    {
      ret = LSM303AGR_ERROR;
    }
    else
    {
      if (pObj->IO.BusType == LSM303AGR_SPI_3WIRES_BUS) /* SPI 3-Wires */
      {
        /* Enable the SPI 3-Wires support only the first time */
        if (pObj->is_initialized == 0U)
        {
          /* Enable SPI 3-Wires on the component */
          uint8_t data = 0x01;

          if (LSM303AGR_ACC_Write_Reg(pObj, LSM303AGR_CTRL_REG4_A, data) != LSM303AGR_OK)
          {
            return LSM303AGR_ERROR;
          }
        }
      }
    }
  }

  return ret;
}

/**
 * @brief  Initialize the LSM303AGR sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_Init(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Capabilities_t * Capabilities)
{
  /* Enable BDU */
  if (lsm303agr_xl_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* FIFO mode selection */
  if (lsm303agr_xl_fifo_mode_set(&(pObj->Ctx), LSM303AGR_BYPASS_MODE) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Select default output data rate. */
  pObj->acc_odr = LSM303AGR_XL_ODR_100Hz;

  /* Output data rate selection - power down. */
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), LSM303AGR_XL_POWER_DOWN) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Full scale selection. */
  if (lsm303agr_xl_full_scale_set(&(pObj->Ctx), LSM303AGR_2g) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  if(new_C_LSM303AGR_rosService())
  {
	  return LSM303AGR_ERROR;
  }

  if(LSM303AGR_OK != LSM303AGR_RosAdvertize(Capabilities))
  {
	  return LSM303AGR_ERROR;
  }


  pObj->is_initialized = 1;

  return LSM303AGR_OK;
}

/**
 * @brief  Deinitialize the LSM303AGR accelerometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_DeInit(LSM303AGR_ACC_Object_t *pObj)
{
  /* Disable the component */
  if (LSM303AGR_ACC_Disable(pObj) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Reset output data rate. */
  pObj->acc_odr = LSM303AGR_XL_POWER_DOWN;
  pObj->is_initialized = 0;

  return LSM303AGR_OK;
}

/**
 * @brief  Read component ID
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_ReadID(LSM303AGR_ACC_Object_t *pObj, uint8_t *Id)
{
  if (lsm303agr_xl_device_id_get(&(pObj->Ctx), Id) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Get LSM303AGR accelerometer sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to LSM303AGR accelerometer sensor capabilities
 * @retval Component status
 */
int32_t LSM303AGR_ACC_GetCapabilities(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Acc          = 1;
  Capabilities->Gyro         = 0;
  Capabilities->Magneto      = 0;
  Capabilities->LowPower     = 0;
  Capabilities->GyroMaxFS    = 0;
  Capabilities->AccMaxFS     = 16;
  Capabilities->MagMaxFS     = 0;
  Capabilities->GyroMaxOdr   = 0.0f;
  Capabilities->AccMaxOdr    = 400.0f;
  Capabilities->MagMaxOdr    = 0.0f;
  return LSM303AGR_OK;
}

/**
 * @brief  Enable the LSM303AGR accelerometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_Enable(LSM303AGR_ACC_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->acc_is_enabled == 1U)
  {
    return LSM303AGR_OK;
  }

  /* Output data rate selection. */
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), pObj->acc_odr) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  pObj->acc_is_enabled = 1;

  return LSM303AGR_OK;
}

/**
 * @brief  Disable the LSM303AGR accelerometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_Disable(LSM303AGR_ACC_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->acc_is_enabled == 0U)
  {
    return LSM303AGR_OK;
  }

  /* Get current output data rate. */
  if (lsm303agr_xl_data_rate_get(&(pObj->Ctx), &pObj->acc_odr) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), LSM303AGR_XL_POWER_DOWN) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  pObj->acc_is_enabled = 0;

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR accelerometer sensor sensitivity
 * @param  pObj the device pObj
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_GetSensitivity(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = LSM303AGR_OK;
  lsm303agr_op_md_a_t op_mode;

  /* Read operative mode from sensor. */
  if (lsm303agr_xl_operating_mode_get(&(pObj->Ctx), &op_mode) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (op_mode)
  {
    case LSM303AGR_HR_12bit:
      if (LSM303AGR_ACC_GetSensitivityHR(pObj, Sensitivity) != LSM303AGR_OK)
      {
        return LSM303AGR_ERROR;
      }
      break;

    case LSM303AGR_NM_10bit:
      if (LSM303AGR_ACC_GetSensitivityNM(pObj, Sensitivity) != LSM303AGR_OK)
      {
        return LSM303AGR_ERROR;
      }
      break;

    case LSM303AGR_LP_8bit:
      if (LSM303AGR_ACC_GetSensitivityLP(pObj, Sensitivity) != LSM303AGR_OK)
      {
        return LSM303AGR_ERROR;
      }
      break;

    default:
      ret = LSM303AGR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM303AGR accelerometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_GetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float *Odr)
{
  int32_t ret = LSM303AGR_OK;
  lsm303agr_op_md_a_t op_mode;
  lsm303agr_odr_a_t odr_low_level;

  /* Read operative mode from sensor. */
  if (lsm303agr_xl_operating_mode_get(&(pObj->Ctx), &op_mode) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Get current output data rate. */
  if (lsm303agr_xl_data_rate_get(&(pObj->Ctx), &odr_low_level) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  if (op_mode == LSM303AGR_LP_8bit) /* LP mode */
  {
    switch (odr_low_level)
    {
      case LSM303AGR_XL_POWER_DOWN:
        *Odr = 0.0f;
        break;

      case LSM303AGR_XL_ODR_1Hz:
        *Odr = 1.0f;
        break;

      case LSM303AGR_XL_ODR_10Hz:
        *Odr = 10.0f;
        break;

      case LSM303AGR_XL_ODR_25Hz:
        *Odr = 25.0f;
        break;

      case LSM303AGR_XL_ODR_50Hz:
        *Odr = 50.0f;
        break;

      case LSM303AGR_XL_ODR_100Hz:
        *Odr = 100.0f;
        break;

      case LSM303AGR_XL_ODR_200Hz:
        *Odr = 200.0f;
        break;

      case LSM303AGR_XL_ODR_400Hz:
        *Odr = 400.0f;
        break;

      case LSM303AGR_XL_ODR_1kHz620_LP:
        *Odr = 1620.0f;
        break;

      case LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP:
        *Odr = 5376.0f;
        break;

      default:
        ret = LSM303AGR_ERROR;
        break;
    }
  }
  else /* HR and NM modes */
  {
    switch (odr_low_level)
    {
      case LSM303AGR_XL_POWER_DOWN:
        *Odr = 0.0f;
        break;

      case LSM303AGR_XL_ODR_1Hz:
        *Odr = 1.0f;
        break;

      case LSM303AGR_XL_ODR_10Hz:
        *Odr = 10.0f;
        break;

      case LSM303AGR_XL_ODR_25Hz:
        *Odr = 25.0f;
        break;

      case LSM303AGR_XL_ODR_50Hz:
        *Odr = 50.0f;
        break;

      case LSM303AGR_XL_ODR_100Hz:
        *Odr = 100.0f;
        break;

      case LSM303AGR_XL_ODR_200Hz:
        *Odr = 200.0f;
        break;

      case LSM303AGR_XL_ODR_400Hz:
        *Odr = 400.0f;
        break;

      case LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP:
        *Odr = 1344.0f;
        break;

      default:
        ret = LSM303AGR_ERROR;
        break;
    }
  }

  return ret;
}

/**
 * @brief  Set the LSM303AGR accelerometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_SetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->acc_is_enabled == 1U)
  {
    return LSM303AGR_ACC_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LSM303AGR_ACC_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LSM303AGR accelerometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_GetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t *FullScale)
{
  int32_t ret = LSM303AGR_OK;
  lsm303agr_fs_a_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fs_low_level) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  switch (fs_low_level)
  {
    case LSM303AGR_2g:
      *FullScale =  2;
      break;

    case LSM303AGR_4g:
      *FullScale =  4;
      break;

    case LSM303AGR_8g:
      *FullScale =  8;
      break;

    case LSM303AGR_16g:
      *FullScale = 16;
      break;

    default:
      ret = LSM303AGR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM303AGR accelerometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_SetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t FullScale)
{
  lsm303agr_fs_a_t new_fs;

  new_fs = (FullScale <= 2) ? LSM303AGR_2g
           : (FullScale <= 4) ? LSM303AGR_4g
           : (FullScale <= 8) ? LSM303AGR_8g
           :                    LSM303AGR_16g;

  if (lsm303agr_xl_full_scale_set(&(pObj->Ctx), new_fs) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR accelerometer sensor raw axes
 * @param  pObj the device pObj
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_GetAxesRaw(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_AxesRaw_t *Value)
{
  int16_t divisor = 1;
  lsm303agr_axis3bit16_t data_raw;
  int32_t ret = LSM303AGR_OK;
  lsm303agr_op_md_a_t op_mode;

  /* Read operative mode from sensor. */
  if (lsm303agr_xl_operating_mode_get(&(pObj->Ctx), &op_mode) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (op_mode)
  {
    case LSM303AGR_HR_12bit:
      divisor = 16;
      break;

    case LSM303AGR_NM_10bit:
      divisor = 64;
      break;

    case LSM303AGR_LP_8bit:
      divisor = 256;
      break;

    default:
      ret = LSM303AGR_ERROR;
      break;
  }

  if (ret == LSM303AGR_ERROR)
  {
    return ret;
  }

  /* Read raw data values. */
  if (lsm303agr_acceleration_raw_get(&(pObj->Ctx), data_raw.u8bit) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Format the data. */
  Value->x = (data_raw.i16bit[0] / divisor);
  Value->y = (data_raw.i16bit[1] / divisor);
  Value->z = (data_raw.i16bit[2] / divisor);

  return ret;
}

/**
 * @brief  Get the LSM303AGR accelerometer sensor axes
 * @param  pObj the device pObj
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_GetAxes(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Axes_t *Acceleration)
{
  LSM303AGR_AxesRaw_t data_raw;
  float sensitivity = 0.0f;

  /* Read raw data values. */
  if (LSM303AGR_ACC_GetAxesRaw(pObj, &data_raw) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Get LSM303AGR actual sensitivity. */
  if (LSM303AGR_ACC_GetSensitivity(pObj, &sensitivity) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Calculate the data. */
  Acceleration->x = (int32_t)((float)((float)data_raw.x * sensitivity));
  Acceleration->y = (int32_t)((float)((float)data_raw.y * sensitivity));
  Acceleration->z = (int32_t)((float)((float)data_raw.z * sensitivity));

  /*	ROS acceleration & publishing */
  LSM303AGR_RosSetAcceleration(pObj, Acceleration);
  LSM303AGR_RosAccelerationPublish(pObj);

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR register value for accelerometer sensor
 * @param  pObj the device pObj
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_Read_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lsm303agr_read_reg(&(pObj->Ctx), Reg, Data, 1) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Set the LSM303AGR register value for accelerometer sensor
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_Write_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lsm303agr_write_reg(&(pObj->Ctx), Reg, &Data, 1) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR ACC data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_Get_DRDY_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *Status)
{
  if (lsm303agr_xl_data_ready_get(&(pObj->Ctx), Status) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR ACC initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_ACC_Get_Init_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return LSM303AGR_ERROR;
  }

  *Status = pObj->is_initialized;

  return LSM303AGR_OK;
}

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_RegisterBusIO(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_IO_t *pIO)
{
  int32_t ret = LSM303AGR_OK;

  if (pObj == NULL)
  {
    ret = LSM303AGR_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadMagRegWrap;
    pObj->Ctx.write_reg = WriteMagRegWrap;
    pObj->Ctx.handle    = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = LSM303AGR_ERROR;
    }
    else if (pObj->IO.Init() != LSM303AGR_OK)
    {
      ret = LSM303AGR_ERROR;
    }
    else
    {
      if (pObj->IO.BusType != LSM303AGR_I2C_BUS) /* If the bus type is not I2C */
      {
        /* Disable I2C interface support only the first time */
        if (pObj->is_initialized == 0U)
        {
          /* Disable I2C interface on the component */
          if (lsm303agr_mag_i2c_interface_set(&(pObj->Ctx), LSM303AGR_I2C_DISABLE) != LSM303AGR_OK)
          {
            return LSM303AGR_ERROR;
          }
        }
      }
    }
  }

  return ret;
}

/**
 * @brief  Initialize the LSM303AGR sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_Init(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Capabilities_t * Capabilities)
{
  /* Enable BDU */
  if (lsm303agr_mag_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Operating mode selection - power down */
  if (lsm303agr_mag_operating_mode_set(&(pObj->Ctx), LSM303AGR_POWER_DOWN) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Output data rate selection */
  if (lsm303agr_mag_data_rate_set(&(pObj->Ctx), LSM303AGR_MG_ODR_100Hz) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Self Test disabled. */
  if (lsm303agr_mag_self_test_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  if(new_C_LSM303AGR_rosService())
  {
	  return LSM303AGR_ERROR;
  }

  if(LSM303AGR_OK != LSM303AGR_RosAdvertize(Capabilities))
  {
	  return LSM303AGR_ERROR;
  }

  pObj->is_initialized = 1;

  return LSM303AGR_OK;
}

/**
 * @brief  Deinitialize the LSM303AGR magnetometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_DeInit(LSM303AGR_MAG_Object_t *pObj)
{
  /* Disable the component */
  if (LSM303AGR_MAG_Disable(pObj) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  pObj->is_initialized = 0;

  return LSM303AGR_OK;
}

/**
 * @brief  Read component ID
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_ReadID(LSM303AGR_MAG_Object_t *pObj, uint8_t *Id)
{
  if (lsm303agr_mag_device_id_get(&(pObj->Ctx), Id) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Get LSM303AGR magnetometer sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to LSM303AGR magnetometer sensor capabilities
 * @retval Component status
 */
int32_t LSM303AGR_MAG_GetCapabilities(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Acc          = 0;
  Capabilities->Gyro         = 0;
  Capabilities->Magneto      = 1;
  Capabilities->LowPower     = 0;
  Capabilities->GyroMaxFS    = 0;
  Capabilities->AccMaxFS     = 0;
  Capabilities->MagMaxFS     = 50;
  Capabilities->GyroMaxOdr   = 0.0f;
  Capabilities->AccMaxOdr    = 0.0f;
  Capabilities->MagMaxOdr    = 100.0f;
  return LSM303AGR_OK;
}

/**
 * @brief Enable the LSM303AGR magnetometer sensor
 * @param pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_Enable(LSM303AGR_MAG_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->mag_is_enabled == 1U)
  {
    return LSM303AGR_OK;
  }

  /* Output data rate selection. */
  if (lsm303agr_mag_operating_mode_set(&(pObj->Ctx), LSM303AGR_CONTINUOUS_MODE) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  pObj->mag_is_enabled = 1;

  return LSM303AGR_OK;
}

/**
 * @brief Disable the LSM303AGR magnetometer sensor
 * @param pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_Disable(LSM303AGR_MAG_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->mag_is_enabled == 0U)
  {
    return LSM303AGR_OK;
  }

  /* Output data rate selection - power down. */
  if (lsm303agr_mag_operating_mode_set(&(pObj->Ctx), LSM303AGR_POWER_DOWN) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  pObj->mag_is_enabled = 0;

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR magnetometer sensor sensitivity
 * @param  pObj the device pObj
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_GetSensitivity(LSM303AGR_MAG_Object_t *pObj, float *Sensitivity)
{
  *Sensitivity = LSM303AGR_MAG_SENSITIVITY_FS_50GAUSS;

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR magnetometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_GetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float *Odr)
{
  int32_t ret = LSM303AGR_OK;
  lsm303agr_mg_odr_m_t odr_low_level;

  /* Get current output data rate. */
  if (lsm303agr_mag_data_rate_get(&(pObj->Ctx), &odr_low_level) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  switch (odr_low_level)
  {
    case LSM303AGR_MG_ODR_10Hz:
      *Odr = 10.0f;
      break;

    case LSM303AGR_MG_ODR_20Hz:
      *Odr = 20.0f;
      break;

    case LSM303AGR_MG_ODR_50Hz:
      *Odr = 50.0f;
      break;

    case LSM303AGR_MG_ODR_100Hz:
      *Odr = 100.0f;
      break;

    default:
      ret = LSM303AGR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM303AGR magnetometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_SetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float Odr)
{
  lsm303agr_mg_odr_m_t new_odr;

  new_odr = (Odr <= 10.000f) ? LSM303AGR_MG_ODR_10Hz
            : (Odr <= 20.000f) ? LSM303AGR_MG_ODR_20Hz
            : (Odr <= 50.000f) ? LSM303AGR_MG_ODR_50Hz
            :                    LSM303AGR_MG_ODR_100Hz;

  if (lsm303agr_mag_data_rate_set(&(pObj->Ctx), new_odr) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}


/**
 * @brief  Get the LSM303AGR magnetometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_GetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t *FullScale)
{
  *FullScale = 50;

  return LSM303AGR_OK;
}

/**
 * @brief  Set the LSM303AGR magnetometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_SetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t FullScale)
{
  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR magnetometer sensor raw axes
 * @param  pObj the device pObj
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_GetAxesRaw(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_AxesRaw_t *Value)
{
  lsm303agr_axis3bit16_t data_raw;

  /* Read raw data values. */
  if (lsm303agr_magnetic_raw_get(&(pObj->Ctx), data_raw.u8bit) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Format the data. */
  Value->x = data_raw.i16bit[0];
  Value->y = data_raw.i16bit[1];
  Value->z = data_raw.i16bit[2];

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR magnetometer sensor axes
 * @param  pObj the device pObj
 * @param  MagneticField pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_GetAxes(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Axes_t * MagneticField)
{
  lsm303agr_axis3bit16_t data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (lsm303agr_magnetic_raw_get(&(pObj->Ctx), data_raw.u8bit) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Get LSM303AGR actual sensitivity. */
  (void)LSM303AGR_MAG_GetSensitivity(pObj, &sensitivity);

  /* Calculate the data. */
  MagneticField->x = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  MagneticField->y = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  MagneticField->z = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  /*	ROS data storage and publishing */
  LSM303AGR_RosSetMagneticField(pObj, MagneticField);
  LSM303AGR_RosMagneticFieldPublish(pObj);

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR register value for magnetic sensor
 * @param  pObj the device pObj
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_Read_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lsm303agr_read_reg(&(pObj->Ctx), Reg, Data, 1) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Set the LSM303AGR register value for magnetic sensor
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_Write_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lsm303agr_write_reg(&(pObj->Ctx), Reg, &Data, 1) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR MAG data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_Get_DRDY_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *Status)
{
  if (lsm303agr_mag_data_ready_get(&(pObj->Ctx), Status) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Get the LSM303AGR MAG initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_MAG_Get_Init_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return LSM303AGR_ERROR;
  }

  *Status = pObj->is_initialized;

  return LSM303AGR_OK;
}

/**
 * @}
 */

/** @defgroup LSM303AGR_Private_Functions LSM303AGR Private Functions
 * @{
 */

/**
 * @brief  Get the LSM303AGR accelerometer sensor sensitivity for HR mode
 * @param  pObj the device pObj
 * @param  Sensitivity pointer to sensitivity
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM303AGR_ACC_GetSensitivityHR(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = LSM303AGR_OK;
  lsm303agr_fs_a_t fullscale;

  /* Read actual full scale selection from sensor. */
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fullscale) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (fullscale)
  {
    case LSM303AGR_2g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_2G_HIGH_RESOLUTION_MODE;
      break;

    case LSM303AGR_4g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_4G_HIGH_RESOLUTION_MODE;
      break;

    case LSM303AGR_8g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_8G_HIGH_RESOLUTION_MODE;
      break;

    case LSM303AGR_16g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_16G_HIGH_RESOLUTION_MODE;
      break;

    default:
      ret = LSM303AGR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM303AGR accelerometer sensor sensitivity for NM mode
 * @param  pObj the device pObj
 * @param  Sensitivity pointer to sensitivity
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM303AGR_ACC_GetSensitivityNM(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = LSM303AGR_OK;
  lsm303agr_fs_a_t fullscale;

  /* Read actual full scale selection from sensor. */
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fullscale) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (fullscale)
  {
    case LSM303AGR_2g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_2G_NORMAL_MODE;
      break;

    case LSM303AGR_4g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_4G_NORMAL_MODE;
      break;

    case LSM303AGR_8g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_8G_NORMAL_MODE;
      break;

    case LSM303AGR_16g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_16G_NORMAL_MODE;
      break;

    default:
      ret = LSM303AGR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM303AGR accelerometer sensor sensitivity for LP mode
 * @param  pObj the device pObj
 * @param  Sensitivity pointer to sensitivity
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM303AGR_ACC_GetSensitivityLP(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = LSM303AGR_OK;
  lsm303agr_fs_a_t fullscale;

  /* Read actual full scale selection from sensor. */
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fullscale) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (fullscale)
  {
    case LSM303AGR_2g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_2G_LOW_POWER_MODE;
      break;

    case LSM303AGR_4g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_4G_LOW_POWER_MODE;
      break;

    case LSM303AGR_8g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_8G_LOW_POWER_MODE;
      break;

    case LSM303AGR_16g:
      *Sensitivity = (float)LSM303AGR_ACC_SENSITIVITY_FS_16G_LOW_POWER_MODE;
      break;

    default:
      ret = LSM303AGR_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM303AGR accelerometer sensor output data rate when enabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Enabled(LSM303AGR_ACC_Object_t *pObj, float Odr)
{
  lsm303agr_odr_a_t new_odr;

  new_odr = (Odr <=    1.0f) ? LSM303AGR_XL_ODR_1Hz
            : (Odr <=   10.0f) ? LSM303AGR_XL_ODR_10Hz
            : (Odr <=   25.0f) ? LSM303AGR_XL_ODR_25Hz
            : (Odr <=   50.0f) ? LSM303AGR_XL_ODR_50Hz
            : (Odr <=  100.0f) ? LSM303AGR_XL_ODR_100Hz
            : (Odr <=  200.0f) ? LSM303AGR_XL_ODR_200Hz
            :                    LSM303AGR_XL_ODR_400Hz;

  /* Output data rate selection. */
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), new_odr) != LSM303AGR_OK)
  {
    return LSM303AGR_ERROR;
  }

  return LSM303AGR_OK;
}

/**
 * @brief  Set the LSM303AGR accelerometer sensor output data rate when disabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Disabled(LSM303AGR_ACC_Object_t *pObj, float Odr)
{
  pObj->acc_odr = (Odr <=    1.0f) ? LSM303AGR_XL_ODR_1Hz
                  : (Odr <=   10.0f) ? LSM303AGR_XL_ODR_10Hz
                  : (Odr <=   25.0f) ? LSM303AGR_XL_ODR_25Hz
                  : (Odr <=   50.0f) ? LSM303AGR_XL_ODR_50Hz
                  : (Odr <=  100.0f) ? LSM303AGR_XL_ODR_100Hz
                  : (Odr <=  200.0f) ? LSM303AGR_XL_ODR_200Hz
                  :                    LSM303AGR_XL_ODR_400Hz;

  return LSM303AGR_OK;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_ACC_Object_t *pObj = (LSM303AGR_ACC_Object_t *)Handle;

  if (pObj->IO.BusType == LSM303AGR_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else   /* SPI 3-Wires */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_ACC_Object_t *pObj = (LSM303AGR_ACC_Object_t *)Handle;

  if (pObj->IO.BusType == LSM303AGR_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else   /* SPI 3-Wires */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_MAG_Object_t *pObj = (LSM303AGR_MAG_Object_t *)Handle;

  if (pObj->IO.BusType == LSM303AGR_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else   /* SPI 3-Wires */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_MAG_Object_t *pObj = (LSM303AGR_MAG_Object_t *)Handle;

  if (pObj->IO.BusType == LSM303AGR_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else   /* SPI 3-Wires */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
