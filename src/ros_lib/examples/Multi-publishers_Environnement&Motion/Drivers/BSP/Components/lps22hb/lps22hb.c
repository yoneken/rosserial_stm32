/**
 ******************************************************************************
 * @file    lps22hb.c
 * @author  MEMS Software Solutions Team
 * @brief   LPS22HB driver file
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
#include "lps22hb.h"
#include "ros/lps22hb_rosWrapper.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @defgroup LPS22HB LPS22HB
 * @{
 */

/** @defgroup LPS22HB_Exported_Variables LPS22HB Exported Variables
 * @{
 */

LPS22HB_CommonDrv_t LPS22HB_COMMON_Driver =
{
  LPS22HB_Init,
  LPS22HB_DeInit,
  LPS22HB_ReadID,
  LPS22HB_GetCapabilities,
};

LPS22HB_PRESS_Drv_t LPS22HB_PRESS_Driver =
{
  LPS22HB_PRESS_Enable,
  LPS22HB_PRESS_Disable,
  LPS22HB_PRESS_GetOutputDataRate,
  LPS22HB_PRESS_SetOutputDataRate,
  LPS22HB_PRESS_GetPressure,
};

LPS22HB_TEMP_Drv_t LPS22HB_TEMP_Driver =
{
  LPS22HB_TEMP_Enable,
  LPS22HB_TEMP_Disable,
  LPS22HB_TEMP_GetOutputDataRate,
  LPS22HB_TEMP_SetOutputDataRate,
  LPS22HB_TEMP_GetTemperature,
};

/**
 * @}
 */

/** @defgroup LPS22HB_Private_Function_Prototypes LPS22HB Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LPS22HB_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
static int32_t LPS22HB_SetOutputDataRate_When_Enabled(LPS22HB_Object_t *pObj, float Odr);
static int32_t LPS22HB_SetOutputDataRate_When_Disabled(LPS22HB_Object_t *pObj, float Odr);
static int32_t LPS22HB_Initialize(LPS22HB_Object_t *pObj);

/**
 * @}
 */

/** @defgroup LPS22HB_Exported_Functions LPS22HB Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_RegisterBusIO(LPS22HB_Object_t *pObj, LPS22HB_IO_t *pIO)
{
  int32_t ret = LPS22HB_OK;

  if (pObj == NULL)
  {
    ret = LPS22HB_ERROR;
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

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = LPS22HB_ERROR;
    }
    else if (pObj->IO.Init() != LPS22HB_OK)
    {
      ret = LPS22HB_ERROR;
    }
    else
    {
      if (pObj->IO.BusType == LPS22HB_SPI_3WIRES_BUS) /* SPI 3-Wires */
      {
        /* Enable the SPI 3-Wires support only the first time */
        if (pObj->is_initialized == 0U)
        {
          /* Enable SPI 3-Wires on the component */
          uint8_t data = 0x01;

          if (LPS22HB_Write_Reg(pObj, LPS22HB_CTRL_REG1, data) != LPS22HB_OK)
          {
            ret = LPS22HB_ERROR;
          }
        }
      }
    }
  }

  return ret;
}

/**
 * @brief  Initialize the LPS22HB sensor
 * @param  pObj the device pObj
 * @param  Capabilities the device capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_Init(LPS22HB_Object_t *pObj, LPS22HB_Capabilities_t * Capabilities)
{
  if (pObj->is_initialized == 0U)
  {
    if (LPS22HB_Initialize(pObj) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }

    if(new_C_LPS22HB_rosService())
    {
    	return LPS22HB_ERROR;
    }

    if(LPS22HB_OK != LPS22HB_RosAdvertize(Capabilities))
    {
    	return LPS22HB_ERROR;
    }
  }

  pObj->is_initialized = 1U;

  return LPS22HB_OK;
}

/**
 * @brief  Deinitialize the LPS22HB sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_DeInit(LPS22HB_Object_t *pObj)
{
  if (pObj->is_initialized == 1U)
  {
    if (LPS22HB_PRESS_Disable(pObj) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }

    if (LPS22HB_TEMP_Disable(pObj) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }

    del_C_LPS22HB_rosService();

  }

  pObj->is_initialized = 0;

  return LPS22HB_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_ReadID(LPS22HB_Object_t *pObj, uint8_t *Id)
{
  if (lps22hb_device_id_get(&(pObj->Ctx), Id) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get LPS22HB sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to LPS22HB sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_GetCapabilities(LPS22HB_Object_t *pObj, LPS22HB_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Humidity    = 0;
  Capabilities->Pressure    = 1;
  Capabilities->Temperature = 1;
  Capabilities->LowPower    = 0;
  Capabilities->HumMaxOdr   = 0.0f;
  Capabilities->TempMaxOdr  = 75.0f;
  Capabilities->PressMaxOdr = 75.0f;
  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_Get_Init_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return LPS22HB_ERROR;
  }

  *Status = pObj->is_initialized;

  return LPS22HB_OK;
}

/**
 * @brief  Enable the LPS22HB pressure sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_PRESS_Enable(LPS22HB_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->press_is_enabled == 1U)
  {
    return LPS22HB_OK;
  }

  /* Output data rate selection. */
  if (lps22hb_data_rate_set(&(pObj->Ctx), pObj->last_odr) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  pObj->press_is_enabled = 1;

  return LPS22HB_OK;
}

/**
 * @brief  Disable the LPS22HB pressure sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_PRESS_Disable(LPS22HB_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->press_is_enabled == 0U)
  {
    return LPS22HB_OK;
  }

  /* Check if the LPS22HB temperature sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (pObj->temp_is_enabled == 0U)
  {
    /* Get current output data rate. */
    if (lps22hb_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }

    /* Output data rate selection - power down. */
    if (lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }
  }

  pObj->press_is_enabled = 0;

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB pressure sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_PRESS_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr)
{
  return LPS22HB_GetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Set the LPS22HB pressure sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_PRESS_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->press_is_enabled == 1U)
  {
    return LPS22HB_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LPS22HB_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LPS22HB pressure value
 * @param  pObj the device pObj
 * @param  Value pointer where the pressure value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_PRESS_GetPressure(LPS22HB_Object_t *pObj, float *Value)
{
  lps22hb_axis1bit32_t data_raw_pressure;

  (void)memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
  if (lps22hb_pressure_raw_get(&(pObj->Ctx), data_raw_pressure.u8bit) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  *Value = lps22hb_from_lsb_to_hpa(data_raw_pressure.i32bit);

  LPS22HB_RosSetPressure(pObj, (double) *Value);
  LPS22HB_RosPressurePublish(pObj);

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB pressure data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_PRESS_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_press_data_ready_get(&(pObj->Ctx), Status) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Enable the LPS22HB temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_TEMP_Enable(LPS22HB_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->temp_is_enabled == 1U)
  {
    return LPS22HB_OK;
  }

  /* Output data rate selection. */
  if (lps22hb_data_rate_set(&(pObj->Ctx), pObj->last_odr) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  pObj->temp_is_enabled = 1;

  return LPS22HB_OK;
}

/**
 * @brief  Disable the LPS22HB temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_TEMP_Disable(LPS22HB_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->temp_is_enabled == 0U)
  {
    return LPS22HB_OK;
  }

  /* Check if the LPS22HB pressure sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (pObj->press_is_enabled == 0U)
  {
    /* Get current output data rate. */
    if (lps22hb_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }

    /* Output data rate selection - power down. */
    if (lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }
  }

  pObj->temp_is_enabled = 0;

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_TEMP_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr)
{
  return LPS22HB_GetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Set the LPS22HB temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_TEMP_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->temp_is_enabled == 1U)
  {
    return LPS22HB_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LPS22HB_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LPS22HB temperature value
 * @param  pObj the device pObj
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_TEMP_GetTemperature(LPS22HB_Object_t *pObj, float *Value)
{
  lps22hb_axis1bit16_t data_raw_temperature;

  (void)memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
  if (lps22hb_temperature_raw_get(&(pObj->Ctx), data_raw_temperature.u8bit) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  *Value = lps22hb_from_lsb_to_degc(data_raw_temperature.i16bit);

  LPS22HB_RosSetTemperature(pObj, (double) *Value);
  LPS22HB_RosTemperaturePublish(pObj);

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB temperature data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_TEMP_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_temp_data_ready_get(&(pObj->Ctx), Status) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB FIFO data level
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Get_Data(LPS22HB_Object_t *pObj, float *Press, float *Temp)
{
  if (LPS22HB_PRESS_GetPressure(pObj, Press) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  if (LPS22HB_Get_Temp(pObj, Temp) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB FIFO threshold
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Get_FTh_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_fth_flag_get(&(pObj->Ctx), Status) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB FIFO full status
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Get_Full_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_data_level_get(&(pObj->Ctx), Status) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  if (*Status == LPS22HB_FIFO_FULL)
  {
    *Status = (uint8_t) 1;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB FIFO data level
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Get_Level(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_data_level_get(&(pObj->Ctx), Status) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB FIFO OVR status
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Get_Ovr_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_ovr_flag_get(&(pObj->Ctx), Status) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Reset the FIFO interrupt
 * @param  pObj the device pObj
 * @param  interrupt The FIFO interrupt to be reset; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Reset_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt)
{
  switch (interrupt)
  {
    case 0:
      if (lps22hb_fifo_threshold_on_int_set(&(pObj->Ctx), PROPERTY_DISABLE) != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
      break;
    case 1:
      if (lps22hb_fifo_full_on_int_set(&(pObj->Ctx), PROPERTY_DISABLE) != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
      break;
    case 2:
      if (lps22hb_fifo_ovr_on_int_set(&(pObj->Ctx), PROPERTY_DISABLE) != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
      break;
    default:
      return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Set the FIFO interrupt
 * @param  pObj the device pObj
 * @param  interrupt The FIFO interrupt to be reset; values: 0 = FTH; 1 = FULL; 2 = OVR
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Set_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt)
{
  switch (interrupt)
  {
    case 0:
      if (lps22hb_fifo_threshold_on_int_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
      break;
    case 1:
      if (lps22hb_fifo_full_on_int_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
      break;
    case 2:
      if (lps22hb_fifo_ovr_on_int_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
      break;
    default:
      return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Set the FIFO mode
 * @param  pObj the device pObj
 * @param  Mode the FIFO mode to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Set_Mode(LPS22HB_Object_t *pObj, uint8_t Mode)
{

  /* Verify that the passed parameter contains one of the valid values */
  switch ((lps22hb_f_mode_t)Mode)
  {
    case LPS22HB_BYPASS_MODE:
    case LPS22HB_FIFO_MODE:
    case LPS22HB_STREAM_MODE:
    case LPS22HB_STREAM_TO_FIFO_MODE:
    case LPS22HB_BYPASS_TO_STREAM_MODE:
    case LPS22HB_BYPASS_TO_FIFO_MODE:
      break;
    default:
      return LPS22HB_ERROR;
  }

  if (lps22hb_fifo_mode_set(&(pObj->Ctx), (lps22hb_f_mode_t)Mode) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Set the LPS22HB FIFO data level
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Set_Watermark_Level(LPS22HB_Object_t *pObj, uint8_t Watermark)
{
  if (lps22hb_fifo_watermark_set(&(pObj->Ctx), Watermark) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Enable the FIFO usage
 * @param  pObj the device pObj
 * @param  Mode the FIFO mode to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_FIFO_Usage(LPS22HB_Object_t *pObj, uint8_t Status)
{

  if (lps22hb_fifo_set(&(pObj->Ctx), Status) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB register value
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_Read_Reg(LPS22HB_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lps22hb_read_reg(&(pObj->Ctx), Reg, Data, 1) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Set the LPS22HB register value
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_Write_Reg(LPS22HB_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lps22hb_write_reg(&(pObj->Ctx), Reg, &Data, 1) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @}
 */

/** @defgroup LPS22HB_Private_Functions LPS22HB Private Functions
 * @{
 */

/**
 * @brief  Get the LPS22HB FIFO get temp data
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_Get_Temp(LPS22HB_Object_t *pObj, float *Data)
{
  uint8_t buffer[2];
  uint32_t tmp = 0;

  /* Read data from LPS22HB. */
  if (lps22hb_temperature_raw_get(&(pObj->Ctx), buffer) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  /* Build the raw tmp */
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];

  *Data = ((float)tmp) / 100.0f;

  return LPS22HB_OK;
}

/**
 * @brief  Get output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS22HB_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr)
{
  int32_t ret = LPS22HB_OK;
  lps22hb_odr_t odr_low_level;

  if (lps22hb_data_rate_get(&(pObj->Ctx), &odr_low_level) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  switch (odr_low_level)
  {
    case LPS22HB_POWER_DOWN:
      *Odr = 0.0f;
      break;

    case LPS22HB_ODR_1_Hz:
      *Odr = 1.0f;
      break;

    case LPS22HB_ODR_10_Hz:
      *Odr = 10.0f;
      break;

    case LPS22HB_ODR_25_Hz:
      *Odr = 25.0f;
      break;

    case LPS22HB_ODR_50_Hz:
      *Odr = 50.0f;
      break;

    case LPS22HB_ODR_75_Hz:
      *Odr = 75.0f;
      break;

    default:
      ret = LPS22HB_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS22HB_SetOutputDataRate_When_Enabled(LPS22HB_Object_t *pObj, float Odr)
{
  lps22hb_odr_t new_odr;

  new_odr = (Odr <=  1.0f) ? LPS22HB_ODR_1_Hz
            : (Odr <= 10.0f) ? LPS22HB_ODR_10_Hz
            : (Odr <= 25.0f) ? LPS22HB_ODR_25_Hz
            : (Odr <= 50.0f) ? LPS22HB_ODR_50_Hz
            :                  LPS22HB_ODR_75_Hz;

  if (lps22hb_data_rate_set(&(pObj->Ctx), new_odr) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  if (lps22hb_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Set output data rate when disabled
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS22HB_SetOutputDataRate_When_Disabled(LPS22HB_Object_t *pObj, float Odr)
{
  pObj->last_odr = (Odr <=  1.0f) ? LPS22HB_ODR_1_Hz
                   : (Odr <= 10.0f) ? LPS22HB_ODR_10_Hz
                   : (Odr <= 25.0f) ? LPS22HB_ODR_25_Hz
                   : (Odr <= 50.0f) ? LPS22HB_ODR_50_Hz
                   :                  LPS22HB_ODR_75_Hz;

  return LPS22HB_OK;
}

/**
 * @brief  Initialize the LPS22HB sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS22HB_Initialize(LPS22HB_Object_t *pObj)
{
  /* Set Power mode */
  if (lps22hb_low_power_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  /* Power down the device */
  if (lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  /* Disable low-pass filter on LPS22HB pressure data */
  if (lps22hb_low_pass_filter_mode_set(&(pObj->Ctx), LPS22HB_LPF_ODR_DIV_9) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  if (lps22hb_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  if (pObj->IO.BusType == LPS22HB_I2C_BUS) /* I2C */
  {
    if (lps22hb_auto_add_inc_set(&(pObj->Ctx), PROPERTY_DISABLE) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }
  }
  else /* SPI 4-Wires or SPI 3-Wires */
  {
    if (lps22hb_auto_add_inc_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS22HB_OK)
    {
      return LPS22HB_ERROR;
    }
  }

  pObj->last_odr = LPS22HB_ODR_25_Hz;

  return LPS22HB_OK;
}

/**
 * @brief  Set the LPS22HB One Shot Mode
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_Set_One_Shot(LPS22HB_Object_t *pObj)
{
  /* Set ODR */
  if(lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  /* Start One Shot Measurement */
  if(lps22hb_one_shoot_trigger_set(&(pObj->Ctx), 1) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Get the LPS22HB One Shot Status
 * @param  pObj the device pObj
 * @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_Get_One_Shot_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  uint8_t p_da;
  uint8_t t_da;

  /* Get DataReady for pressure */
  if(lps22hb_press_data_ready_get(&(pObj->Ctx), &p_da) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  /* Get DataReady for temperature */
  if(lps22hb_temp_data_ready_get(&(pObj->Ctx), &t_da) != LPS22HB_OK)
  {
    return LPS22HB_ERROR;
  }

  if(p_da && t_da)
  {
    *Status = 1;
  }
  else
  {
    *Status = 0;
  }

  return LPS22HB_OK;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  uint16_t i;
  int32_t ret = LPS22HB_OK;
  LPS22HB_Object_t *pObj = (LPS22HB_Object_t *)Handle;

  if (pObj->IO.BusType == LPS22HB_I2C_BUS) /* I2C */
  {
    for (i = 0; i < Length; i++)
    {
      ret = pObj->IO.ReadReg(pObj->IO.Address, (Reg + i), &pData[i], 1);
      if (ret != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
    }

    return ret;
  }
  else /* SPI 4-Wires or SPI 3-Wires */
  {
    return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
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
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  uint16_t i;
  int32_t ret = LPS22HB_OK;
  LPS22HB_Object_t *pObj = (LPS22HB_Object_t *)Handle;

  if (pObj->IO.BusType == LPS22HB_I2C_BUS) /* I2C */
  {
    for (i = 0; i < Length; i++)
    {
      ret = pObj->IO.WriteReg(pObj->IO.Address, (Reg + i), &pData[i], 1);
      if (ret != LPS22HB_OK)
      {
        return LPS22HB_ERROR;
      }
    }

    return ret;
  }
  else /* SPI 4-Wires or SPI 3-Wires */
  {
    return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
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
