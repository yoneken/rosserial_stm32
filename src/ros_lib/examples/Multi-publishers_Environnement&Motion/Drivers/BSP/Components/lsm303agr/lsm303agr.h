/**
 ******************************************************************************
 * @file    lsm303agr.h
 * @author  MEMS Software Solutions Team
 * @brief   LSM303AGR header driver file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LSM303AGR_H
#define LSM303AGR_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "lsm303agr_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup LSM303AGR LSM303AGR
 * @{
 */

/** @defgroup LSM303AGR_Exported_Types LSM303AGR Exported Types
 * @{
 */

typedef int32_t (*LSM303AGR_Init_Func)(void);
typedef int32_t (*LSM303AGR_DeInit_Func)(void);
typedef int32_t (*LSM303AGR_GetTick_Func)(void);
typedef int32_t (*LSM303AGR_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LSM303AGR_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LSM303AGR_Init_Func          Init;
  LSM303AGR_DeInit_Func        DeInit;
  uint32_t                     BusType; /*0 means I2C, 1 means SPI-3-Wires */
  uint8_t                      Address;
  LSM303AGR_WriteReg_Func      WriteReg;
  LSM303AGR_ReadReg_Func       ReadReg;
  LSM303AGR_GetTick_Func       GetTick;
} LSM303AGR_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LSM303AGR_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM303AGR_Axes_t;

typedef struct
{
  LSM303AGR_IO_t        IO;
  stmdev_ctx_t          Ctx;
  uint8_t               is_initialized;
  uint8_t               acc_is_enabled;
  lsm303agr_odr_a_t     acc_odr;
} LSM303AGR_ACC_Object_t;

typedef struct
{
  LSM303AGR_IO_t        IO;
  stmdev_ctx_t          Ctx;
  uint8_t               is_initialized;
  uint8_t               mag_is_enabled;
} LSM303AGR_MAG_Object_t;

typedef struct
{
  uint8_t   Acc;
  uint8_t   Gyro;
  uint8_t   Magneto;
  uint8_t   LowPower;
  uint32_t  GyroMaxFS;
  uint32_t  AccMaxFS;
  uint32_t  MagMaxFS;
  float     GyroMaxOdr;
  float     AccMaxOdr;
  float     MagMaxOdr;
} LSM303AGR_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LSM303AGR_ACC_Object_t *, LSM303AGR_Capabilities_t *);
  int32_t (*DeInit)(LSM303AGR_ACC_Object_t *);
  int32_t (*ReadID)(LSM303AGR_ACC_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM303AGR_ACC_Object_t *, LSM303AGR_Capabilities_t *);
} LSM303AGR_ACC_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM303AGR_ACC_Object_t *);
  int32_t (*Disable)(LSM303AGR_ACC_Object_t *);
  int32_t (*GetSensitivity)(LSM303AGR_ACC_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM303AGR_ACC_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM303AGR_ACC_Object_t *, float);
  int32_t (*GetFullScale)(LSM303AGR_ACC_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM303AGR_ACC_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM303AGR_ACC_Object_t *, LSM303AGR_Axes_t *);
  int32_t (*GetAxesRaw)(LSM303AGR_ACC_Object_t *, LSM303AGR_AxesRaw_t *);
} LSM303AGR_ACC_Drv_t;

typedef struct
{
  int32_t (*Init)(LSM303AGR_MAG_Object_t *, LSM303AGR_Capabilities_t * Capabilities);
  int32_t (*DeInit)(LSM303AGR_MAG_Object_t *);
  int32_t (*ReadID)(LSM303AGR_MAG_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM303AGR_MAG_Object_t *, LSM303AGR_Capabilities_t *);
} LSM303AGR_MAG_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM303AGR_MAG_Object_t *);
  int32_t (*Disable)(LSM303AGR_MAG_Object_t *);
  int32_t (*GetSensitivity)(LSM303AGR_MAG_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM303AGR_MAG_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM303AGR_MAG_Object_t *, float);
  int32_t (*GetFullScale)(LSM303AGR_MAG_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM303AGR_MAG_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM303AGR_MAG_Object_t *, LSM303AGR_Axes_t *);
  int32_t (*GetAxesRaw)(LSM303AGR_MAG_Object_t *, LSM303AGR_AxesRaw_t *);
} LSM303AGR_MAG_Drv_t;

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} lsm303agr_axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} lsm303agr_axis1bit16_t;

typedef union{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} lsm303agr_axis3bit32_t;

typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} lsm303agr_axis1bit32_t;

/**
 * @}
 */

/** @defgroup LSM303AGR_Exported_Constants LSM303AGR Exported Constants
 * @{
 */

#define LSM303AGR_OK                     0
#define LSM303AGR_ERROR                 -1

#define LSM303AGR_I2C_BUS               0U
#define LSM303AGR_SPI_3WIRES_BUS        1U

#define LSM303AGR_ACC_SENSITIVITY_FS_2G_NORMAL_MODE               3.900f  /**< Sensitivity value for 2 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_2G_HIGH_RESOLUTION_MODE      0.980f  /**< Sensitivity value for 2 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_2G_LOW_POWER_MODE           15.630f  /**< Sensitivity value for 2 g full scale and low power mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_4G_NORMAL_MODE               7.820f  /**< Sensitivity value for 4 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_4G_HIGH_RESOLUTION_MODE      1.950f  /**< Sensitivity value for 4 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_4G_LOW_POWER_MODE           31.260f  /**< Sensitivity value for 4 g full scale and low power mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_8G_NORMAL_MODE              15.630f  /**< Sensitivity value for 8 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_8G_HIGH_RESOLUTION_MODE      3.900f  /**< Sensitivity value for 8 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_8G_LOW_POWER_MODE           62.520f  /**< Sensitivity value for 8 g full scale and low power mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_16G_NORMAL_MODE             46.900f  /**< Sensitivity value for 16 g full scale and normal mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_16G_HIGH_RESOLUTION_MODE    11.720f  /**< Sensitivity value for 16 g full scale and high resolution mode [mg/LSB] */
#define LSM303AGR_ACC_SENSITIVITY_FS_16G_LOW_POWER_MODE         187.580f  /**< Sensitivity value for 16 g full scale and low power mode [mg/LSB] */

#define LSM303AGR_MAG_SENSITIVITY_FS_50GAUSS  1.500f  /**< Sensitivity value for 50 gauss full scale [mgauss/LSB] */

/**
 * @}
 */

/** @defgroup LSM303AGR_Exported_Functions LSM303AGR Exported Functions
 * @{
 */

int32_t LSM303AGR_ACC_RegisterBusIO(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_IO_t *pIO);
int32_t LSM303AGR_ACC_Init(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Capabilities_t * Capabilities);
int32_t LSM303AGR_ACC_DeInit(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_ReadID(LSM303AGR_ACC_Object_t *pObj, uint8_t *Id);
int32_t LSM303AGR_ACC_GetCapabilities(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities);

int32_t LSM303AGR_ACC_Enable(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_Disable(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_GetSensitivity(LSM303AGR_ACC_Object_t *pObj, float *sensitivity);
int32_t LSM303AGR_ACC_GetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float *odr);
int32_t LSM303AGR_ACC_SetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float odr);
int32_t LSM303AGR_ACC_GetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t *fullscale);
int32_t LSM303AGR_ACC_SetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t fullscale);
int32_t LSM303AGR_ACC_GetAxes(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Axes_t *acceleration);
int32_t LSM303AGR_ACC_GetAxesRaw(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_AxesRaw_t *value);

int32_t LSM303AGR_ACC_Read_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t LSM303AGR_ACC_Write_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t LSM303AGR_ACC_Get_DRDY_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *status);
int32_t LSM303AGR_ACC_Get_Init_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *status);

int32_t LSM303AGR_MAG_RegisterBusIO(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_IO_t *pIO);
int32_t LSM303AGR_MAG_Init(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Capabilities_t * Capabilities);
int32_t LSM303AGR_MAG_DeInit(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_ReadID(LSM303AGR_MAG_Object_t *pObj, uint8_t *id);
int32_t LSM303AGR_MAG_GetCapabilities(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities);

int32_t LSM303AGR_MAG_Enable(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_Disable(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_GetSensitivity(LSM303AGR_MAG_Object_t *pObj, float *sensitivity);
int32_t LSM303AGR_MAG_GetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float *odr);
int32_t LSM303AGR_MAG_SetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float odr);
int32_t LSM303AGR_MAG_GetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t *fullscale);
int32_t LSM303AGR_MAG_SetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t fullscale);
int32_t LSM303AGR_MAG_GetAxes(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Axes_t *magnetic_field);
int32_t LSM303AGR_MAG_GetAxesRaw(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_AxesRaw_t *value);

int32_t LSM303AGR_MAG_Read_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t LSM303AGR_MAG_Write_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t LSM303AGR_MAG_Get_DRDY_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *status);
int32_t LSM303AGR_MAG_Get_Init_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *status);

/**
 * @}
 */

/** @addtogroup LSM303AGR_Exported_Variables LSM303AGR Exported Variables
 * @{
 */

extern LSM303AGR_ACC_CommonDrv_t LSM303AGR_ACC_COMMON_Driver;
extern LSM303AGR_ACC_Drv_t LSM303AGR_ACC_Driver;
extern LSM303AGR_MAG_CommonDrv_t LSM303AGR_MAG_COMMON_Driver;
extern LSM303AGR_MAG_Drv_t LSM303AGR_MAG_Driver;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

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
