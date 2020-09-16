/**
 ******************************************************************************
 * @file    iks01a2_motion_sensors.h
 * @author  MEMS Software Solutions Team
 * @brief   This file provides a set of functions needed to manage the motion sensors
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
#ifndef IKS01A2_MOTION_SENSOR_H
#define IKS01A2_MOTION_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "iks01a2_conf.h"
#include "motion_sensor.h"

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0
#define USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0          1
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0
#define USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0    1
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0
#define USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0    1
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0
#define USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0        0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0
#define USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0         0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0
#define USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0          0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0
#define USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0    0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0
#define USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0    0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0
#define USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0        0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0
#define USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0         0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0
#define USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0         0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0
#define USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0          0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0
#define USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0          0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0
#define USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0         0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0
#define USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0         0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0
#define USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0          0
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0
#define USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0          0
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
#include "lsm6dsl.h"
#endif

#if ((USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1) || (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1))
#include "lsm303agr.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 == 1)
#include "asm330lhh.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 == 1)
#include "iis2dlpc.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 == 1)
#include "iis2mdc.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 == 1)
#include "ism303dac.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 == 1)
#include "ism303dac.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
#include "ism330dlc.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 == 1)
#include "lis2dh12.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 == 1)
#include "lis2dw12.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 == 1)
#include "lis2mdl.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 == 1)
#include "lsm6dso.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 == 1)
#include "lsm6dsox.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0 == 1)
#include "ais2dw12.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0 == 1)
#include "lis3mdl.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0 == 1)
#include "lsm6dsr.h"
#endif


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup IKS01A2 IKS01A2
 * @{
 */

/** @addtogroup IKS01A2_MOTION_SENSOR IKS01A2 MOTION SENSOR
 * @{
 */

/** @defgroup IKS01A2_MOTION_SENSOR_Exported_Types IKS01A2 MOTION SENSOR Exported Types
 * @{
 */

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} IKS01A2_MOTION_SENSOR_Axes_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} IKS01A2_MOTION_SENSOR_AxesRaw_t;

/* Motion Sensor instance Info */
typedef struct
{
  uint8_t  Acc;
  uint8_t  Gyro;
  uint8_t  Magneto;
  uint8_t  LowPower;
  uint32_t GyroMaxFS;
  uint32_t AccMaxFS;
  uint32_t MagMaxFS;
  float    GyroMaxOdr;
  float    AccMaxOdr;
  float    MagMaxOdr;
} IKS01A2_MOTION_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} IKS01A2_MOTION_SENSOR_Ctx_t;

/**
 * @}
 */

/** @defgroup IKS01A2_MOTION_SENSOR_Exported_Constants IKS01A2 MOTION SENSOR Exported Constants
 * @{
 */

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
#define IKS01A2_LSM6DSL_0 0
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
#define IKS01A2_LSM303AGR_ACC_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
#define IKS01A2_LSM303AGR_MAG_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 == 1)
#define IKS01A2_ASM330LHH_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                             USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                             USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 == 1)
#define IKS01A2_IIS2DLPC_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 == 1)
#define IKS01A2_IIS2MDC_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                           USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 == 1)
#define IKS01A2_ISM303DAC_ACC_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 == 1)
#define IKS01A2_ISM303DAC_MAG_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                                 USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
#define IKS01A2_ISM330DLC_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                             USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                             USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                             USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                             USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                             USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                             USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                             USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 == 1)
#define IKS01A2_LIS2DH12_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 == 1)
#define IKS01A2_LIS2DW12_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 == 1)
#define IKS01A2_LIS2MDL_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                           USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 == 1)
#define IKS01A2_LSM6DSO_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                           USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                           USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 + \
                           USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 == 1)
#define IKS01A2_LSM6DSOX_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0 == 1)
#define IKS01A2_AIS2DW12_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0)
                            
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0 == 1)
#define IKS01A2_LIS3MDL_0  (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 + \
                            USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0)
                            
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0 == 1)
#define IKS01A2_LSM6DSR_0  (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                            USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 + \
                            USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0 + \
                            USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0)
                            
#endif

#ifndef MOTION_GYRO
#define MOTION_GYRO             1U
#endif
#ifndef MOTION_ACCELERO
#define MOTION_ACCELERO         2U
#endif
#ifndef MOTION_MAGNETO
#define MOTION_MAGNETO          4U
#endif

#define IKS01A2_MOTION_FUNCTIONS_NBR    3U
#define IKS01A2_MOTION_INSTANCES_NBR    (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0 + \
                                         USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0)

#if (IKS01A2_MOTION_INSTANCES_NBR == 0)
#warning "No motion sensor instance has been selected"
#endif

/**
 * @}
 */

/** @addtogroup IKS01A2_MOTION_SENSOR_Exported_Functions IKS01A2_MOTION_SENSOR Exported Functions
 * @{
 */

int32_t IKS01A2_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t IKS01A2_MOTION_SENSOR_DeInit(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_GetCapabilities(uint32_t Instance, IKS01A2_MOTION_SENSOR_Capabilities_t *Capabilities);
int32_t IKS01A2_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t IKS01A2_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, IKS01A2_MOTION_SENSOR_Axes_t *Axes);
int32_t IKS01A2_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, IKS01A2_MOTION_SENSOR_AxesRaw_t *Axes);
int32_t IKS01A2_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, double *Sensitivity);
int32_t IKS01A2_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, double *Odr);
int32_t IKS01A2_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, double Odr);
int32_t IKS01A2_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale);
int32_t IKS01A2_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale);

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

#ifdef __cplusplus
}
#endif

#endif /* IKS01A2_MOTION_SENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
