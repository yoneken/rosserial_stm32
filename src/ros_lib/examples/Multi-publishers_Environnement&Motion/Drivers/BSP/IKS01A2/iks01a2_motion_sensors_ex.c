/**
 ******************************************************************************
 * @file    iks01a2_motion_sensors_ex.c
 * @author  MEMS Software Solutions Team
 * @brief   This file provides a set of extended functions needed to manage the motion sensors
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
#include "iks01a2_motion_sensors_ex.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup IKS01A2 IKS01A2
 * @{
 */

/** @defgroup IKS01A2_MOTION_SENSOR_EX IKS01A2 MOTION SENSOR EX
 * @{
 */

/** @defgroup IKS01A2_MOTION_SENSOR_EX_Imported_Variables IKS01A2 MOTION SENSOR EX Imported Variables
 * @{
 */

extern void *MotionCompObj[IKS01A2_MOTION_INSTANCES_NBR];

/**
 * @}
 */

/** @defgroup IKS01A2_MOTION_SENSOR_EX_Exported_Functions IKS01A2 MOTION SENSOR EX Exported Functions
 * @{
 */

/**
 * @brief  Enable HP filtering (available only for ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  Cutoff frequency
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_HP_Filter(uint32_t Instance, uint8_t CutOff)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Enable_HP_Filter(MotionCompObj[Instance], (ism330dlc_hpcf_xl_t)CutOff) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set DRDY interrupt on INT1 pin (available only for ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  Status DRDY interrupt on INT1 pin
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_INT1_DRDY(uint32_t Instance, uint8_t Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_INT1_DRDY(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_Set_INT1_Drdy(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set DRDY interrupt on INT2 pin (available only for ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  Status DRDY interrupt on INT1 pin
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_INT2_DRDY(uint32_t Instance, uint8_t Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_Set_INT2_Drdy(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set DRDY mode (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  Status of DRDY mode
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(uint32_t Instance, uint8_t Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_Set_DRDY_Mode(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_Set_Drdy_Mode(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Clear DRDY (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Clear_DRDY(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    ISM330DLC_Axes_t TempAcceleration;

    case IKS01A2_ISM330DLC_0:

      if (ISM330DLC_ACC_GetAxes(MotionCompObj[Instance], &TempAcceleration) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable 6D Orientation (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  IntPin the interrupt pin to be used
 * @note   This function sets the LSM6DSL accelerometer ODR to 416Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_6D_Orientation(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_6D_Orientation(MotionCompObj[Instance], (LSM6DSL_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Enable_6D_Orientation(MotionCompObj[Instance], (ISM330DLC_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable 6D Orientation (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_6D_Orientation(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_6D_Orientation(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Disable_6D_Orientation(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the 6D orientation threshold (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  Threshold the threshold to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_6D_Orientation_Threshold(uint32_t Instance, uint8_t Threshold)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_6D_Orientation_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_6D_Orientation_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable the free fall detection (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  IntPin the interrupt pin to be used
 * @note   This function sets the LSM6DSL accelerometer ODR to 416Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_Free_Fall_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Free_Fall_Detection(MotionCompObj[Instance], (LSM6DSL_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Enable_Free_Fall_Detection(MotionCompObj[Instance], (ISM330DLC_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable the free fall detection (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_Free_Fall_Detection(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_Free_Fall_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Disable_Free_Fall_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the free fall detection threshold (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  Threshold the threshold to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Free_Fall_Threshold(uint32_t Instance, uint8_t Threshold)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Free_Fall_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Free_Fall_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the free fall detection duration (available only for LSM6DSL, ISM330DLC sensor)
 * @param  Instance the device instance
 * @param  Duration the duration to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Free_Fall_Duration(uint32_t Instance, uint8_t Duration)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Free_Fall_Duration(MotionCompObj[Instance], Duration) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Free_Fall_Duration(MotionCompObj[Instance], Duration) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable the pedometer detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @note   This function sets the LSM6DSL accelerometer ODR to 26Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_Pedometer(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Pedometer(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable the pedometer detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_Pedometer(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_Pedometer(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the pedometer threshold (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Threshold the threshold to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Pedometer_Threshold(uint32_t Instance, uint8_t Threshold)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Pedometer_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Reset step counter (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @note   This function sets the LSM6DSL accelerometer ODR to 26Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Reset_Step_Counter(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Step_Counter_Reset(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable the single tap detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  IntPin the interrupt pin to be used
 * @note   This function sets the LSM6DSL accelerometer ODR to 416Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_Single_Tap_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Single_Tap_Detection(MotionCompObj[Instance], (LSM6DSL_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Enable_Single_Tap_Detection(MotionCompObj[Instance], (ISM330DLC_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable the single tap detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_Single_Tap_Detection(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_Single_Tap_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Disable_Single_Tap_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable the double tap detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  IntPin the interrupt pin to be used
 * @note   This function sets the LSM6DSL accelerometer ODR to 416Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_Double_Tap_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Double_Tap_Detection(MotionCompObj[Instance], (LSM6DSL_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Enable_Double_Tap_Detection(MotionCompObj[Instance], (ISM330DLC_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable the double tap detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_Double_Tap_Detection(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_Double_Tap_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Disable_Double_Tap_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the tap threshold (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Threshold the threshold to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Threshold(uint32_t Instance, uint8_t Threshold)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Tap_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Tap_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the tap shock time (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Time the tap shock time to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Shock_Time(uint32_t Instance, uint8_t Time)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Tap_Shock_Time(MotionCompObj[Instance], Time) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Tap_Shock_Time(MotionCompObj[Instance], Time) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the tap quiet time (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Time the tap quiet time to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Quiet_Time(uint32_t Instance, uint8_t Time)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Tap_Quiet_Time(MotionCompObj[Instance], Time) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Tap_Quiet_Time(MotionCompObj[Instance], Time) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the tap duration time (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Time the tap duration time to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Duration_Time(uint32_t Instance, uint8_t Time)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Tap_Duration_Time(MotionCompObj[Instance], Time) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Tap_Duration_Time(MotionCompObj[Instance], Time) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable the tilt detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  IntPin the interrupt pin to be used
 * @note   This function sets the LSM6DSL accelerometer ODR to 26Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_Tilt_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Tilt_Detection(MotionCompObj[Instance], (LSM6DSL_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Enable_Tilt_Detection(MotionCompObj[Instance], (ISM330DLC_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable the tilt detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_Tilt_Detection(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_Tilt_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Disable_Tilt_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable the wake up detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  IntPin the interrupt pin to be used
 * @note   This function sets the LSM6DSL accelerometer ODR to 416Hz and the LSM6DSL accelerometer full scale to 2g
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_Wake_Up_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Wake_Up_Detection(MotionCompObj[Instance], (LSM6DSL_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Enable_Wake_Up_Detection(MotionCompObj[Instance], (ISM330DLC_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable the wake up detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_Wake_Up_Detection(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_Wake_Up_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Disable_Wake_Up_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the wake up detection threshold (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Threshold the threshold to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Wake_Up_Threshold(uint32_t Instance, uint8_t Threshold)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Wake_Up_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Wake_Up_Threshold(MotionCompObj[Instance], Threshold) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the wake up detection duration (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Duration the duration to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Wake_Up_Duration(uint32_t Instance, uint8_t Duration)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Wake_Up_Duration(MotionCompObj[Instance], Duration) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Set_Wake_Up_Duration(MotionCompObj[Instance], Duration) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Enable the inactivity detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  IntPin the interrupt pin to be used
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Enable_Inactivity_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Enable_Inactivity_Detection(MotionCompObj[Instance], (LSM6DSL_SensorIntPin_t)IntPin) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Disable the inactivity detection (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Disable_Inactivity_Detection(uint32_t Instance)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Disable_Inactivity_Detection(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the sleep duration (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Duration the duration to be set
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_Sleep_Duration(uint32_t Instance, uint8_t Duration)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Set_Sleep_Duration(MotionCompObj[Instance], Duration) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get the status of all hardware events (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Status the pointer to the status of all hardware events
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_Event_Status(uint32_t Instance, IKS01A2_MOTION_SENSOR_Event_Status_t *Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      if (LSM6DSL_ACC_Get_Event_Status(MotionCompObj[Instance], (LSM6DSL_Event_Status_t *)(void *)Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      if (ISM330DLC_ACC_Get_Event_Status(MotionCompObj[Instance], (ISM330DLC_Event_Status_t *)(void *)Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get the status of data ready bit (available only for LSM6DSL, LSM303AGR, ASM330LHH sensors)
 * @param  Instance the device instance
 * @param  Function Motion sensor function. Could be:
 *         - MOTION_ACCELERO or MOTION_GYRO for instance IKS01A2_LSM6DSL_0
 *         - MOTION_ACCELERO for instance IKS01A2_LSM303AGR_ACC_0
 *         - MOTION_MAGNETO for instance IKS01A2_LSM303AGR_MAG_0
 * @param  Status the pointer to the status
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_DRDY_Status(uint32_t Instance, uint32_t Function, uint8_t *Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM6DSL_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (LSM6DSL_GYRO_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
    case IKS01A2_LSM303AGR_ACC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM303AGR_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
    case IKS01A2_LSM303AGR_MAG_0:
      if ((Function & MOTION_MAGNETO) == MOTION_MAGNETO)
      {
        if (LSM303AGR_MAG_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 == 1)
    case IKS01A2_ASM330LHH_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (ASM330LHH_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (ASM330LHH_GYRO_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 == 1)
    case IKS01A2_IIS2DLPC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (IIS2DLPC_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 == 1)
    case IKS01A2_IIS2MDC_0:
      if ((Function & MOTION_MAGNETO) == MOTION_MAGNETO)
      {
        if (IIS2MDC_MAG_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 == 1)
    case IKS01A2_ISM303DAC_ACC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (ISM303DAC_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 == 1)
    case IKS01A2_ISM303DAC_MAG_0:
      if ((Function & MOTION_MAGNETO) == MOTION_MAGNETO)
      {
        if (ISM303DAC_MAG_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (ISM330DLC_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (ISM330DLC_GYRO_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 == 1)
    case IKS01A2_LIS2DH12_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LIS2DH12_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 == 1)
    case IKS01A2_LIS2DW12_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LIS2DW12_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 == 1)
    case IKS01A2_LIS2MDL_0:
      if ((Function & MOTION_MAGNETO) == MOTION_MAGNETO)
      {
        if (LIS2MDL_MAG_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 == 1)
    case IKS01A2_LSM6DSO_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM6DSO_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (LSM6DSO_GYRO_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 == 1)
    case IKS01A2_LSM6DSOX_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM6DSOX_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (LSM6DSOX_GYRO_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0 == 1)
    case IKS01A2_AIS2DW12_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (AIS2DW12_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0 == 1)
    case IKS01A2_LIS3MDL_0:
      if ((Function & MOTION_MAGNETO) == MOTION_MAGNETO)
      {
        if (LIS3MDL_MAG_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0 == 1)
    case IKS01A2_LSM6DSR_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM6DSR_ACC_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (LSM6DSR_GYRO_Get_DRDY_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get 6D Orientation XL (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  xl the pointer to the 6D orientation XL axis
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XL(uint32_t Instance, uint8_t *xl)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Get_6D_Orientation_XL(MotionCompObj[Instance], xl) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Get_6D_Orientation_XL(MotionCompObj[Instance], xl) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get 6D Orientation XH (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  xh the pointer to the 6D orientation XH axis
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XH(uint32_t Instance, uint8_t *xh)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Get_6D_Orientation_XH(MotionCompObj[Instance], xh) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Get_6D_Orientation_XH(MotionCompObj[Instance], xh) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get 6D Orientation YL (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  yl the pointer to the 6D orientation YL axis
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YL(uint32_t Instance, uint8_t *yl)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Get_6D_Orientation_YL(MotionCompObj[Instance], yl) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Get_6D_Orientation_YL(MotionCompObj[Instance], yl) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get 6D Orientation YH (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  yh the pointer to the 6D orientation YH axis
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YH(uint32_t Instance, uint8_t *yh)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Get_6D_Orientation_YH(MotionCompObj[Instance], yh) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Get_6D_Orientation_YH(MotionCompObj[Instance], yh) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get 6D Orientation ZL (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  zl the pointer to the 6D orientation ZL axis
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZL(uint32_t Instance, uint8_t *zl)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Get_6D_Orientation_ZL(MotionCompObj[Instance], zl) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Get_6D_Orientation_ZL(MotionCompObj[Instance], zl) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get 6D Orientation ZH (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  zh the pointer to the 6D orientation ZH axis
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZH(uint32_t Instance, uint8_t *zh)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Get_6D_Orientation_ZH(MotionCompObj[Instance], zh) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_ACC_Get_6D_Orientation_ZH(MotionCompObj[Instance], zh) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get step count (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  StepCount number of steps
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Get_Step_Count(uint32_t Instance, uint16_t *StepCount)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_ACC_Get_Step_Count(MotionCompObj[Instance], StepCount) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get the register value (available only for LSM6DSL, LSM303AGR, ASM330LHH sensors)
 * @param  Instance the device instance
 * @param  Reg address to be read
 * @param  Data pointer where the value is written to
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Read_Register(uint32_t Instance, uint8_t Reg, uint8_t *Data)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
    case IKS01A2_LSM303AGR_ACC_0:
      if (LSM303AGR_ACC_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
    case IKS01A2_LSM303AGR_MAG_0:
      if (LSM303AGR_MAG_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 == 1)
    case IKS01A2_ASM330LHH_0:
      if (ASM330LHH_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 == 1)
    case IKS01A2_IIS2DLPC_0:
      if (IIS2DLPC_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 == 1)
    case IKS01A2_IIS2MDC_0:
      if (IIS2MDC_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 == 1)
    case IKS01A2_ISM303DAC_ACC_0:
      if (ISM303DAC_ACC_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 == 1)
    case IKS01A2_ISM303DAC_MAG_0:
      if (ISM303DAC_MAG_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 == 1)
    case IKS01A2_LIS2DH12_0:
      if (LIS2DH12_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 == 1)
    case IKS01A2_LIS2DW12_0:
      if (LIS2DW12_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 == 1)
    case IKS01A2_LIS2MDL_0:
      if (LIS2MDL_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 == 1)
    case IKS01A2_LSM6DSO_0:
      if (LSM6DSO_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 == 1)
    case IKS01A2_LSM6DSOX_0:
      if (LSM6DSOX_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0 == 1)
    case IKS01A2_AIS2DW12_0:
      if (AIS2DW12_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0 == 1)
    case IKS01A2_LIS3MDL_0:
      if (LIS3MDL_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0 == 1)
    case IKS01A2_LSM6DSR_0:
      if (LSM6DSR_Read_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set the register value (available only for LSM6DSL, LSM303AGR, ASM330LHH sensors)
 * @param  Instance the device instance
 * @param  Reg address to be read
 * @param  Data value to be written
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Write_Register(uint32_t Instance, uint8_t Reg, uint8_t Data)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
    case IKS01A2_LSM303AGR_ACC_0:
      if (LSM303AGR_ACC_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
    case IKS01A2_LSM303AGR_MAG_0:
      if (LSM303AGR_MAG_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ASM330LHH_0 == 1)
    case IKS01A2_ASM330LHH_0:
      if (ASM330LHH_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2DLPC_0 == 1)
    case IKS01A2_IIS2DLPC_0:
      if (IIS2DLPC_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_IIS2MDC_0 == 1)
    case IKS01A2_IIS2MDC_0:
      if (IIS2MDC_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_ACC_0 == 1)
    case IKS01A2_ISM303DAC_ACC_0:
      if (ISM303DAC_ACC_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM303DAC_MAG_0 == 1)
    case IKS01A2_ISM303DAC_MAG_0:
      if (ISM303DAC_MAG_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DH12_0 == 1)
    case IKS01A2_LIS2DH12_0:
      if (LIS2DH12_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2DW12_0 == 1)
    case IKS01A2_LIS2DW12_0:
      if (LIS2DW12_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS2MDL_0 == 1)
    case IKS01A2_LIS2MDL_0:
      if (LIS2MDL_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSO_0 == 1)
    case IKS01A2_LSM6DSO_0:
      if (LSM6DSO_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSOX_0 == 1)
    case IKS01A2_LSM6DSOX_0:
      if (LSM6DSOX_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_AIS2DW12_0 == 1)
    case IKS01A2_AIS2DW12_0:
      if (AIS2DW12_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LIS3MDL_0 == 1)
    case IKS01A2_LIS3MDL_0:
      if (LIS3MDL_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSR_0 == 1)
    case IKS01A2_LSM6DSR_0:
      if (LSM6DSR_Write_Reg(MotionCompObj[Instance], Reg, Data) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get number of unread FIFO samples (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  NumSamples number of unread FIFO samples
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Num_Samples(uint32_t Instance, uint16_t *NumSamples)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Get_Num_Samples(MotionCompObj[Instance], NumSamples) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Get_Num_Samples(MotionCompObj[Instance], NumSamples) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get FIFO full status (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Status FIFO full status
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Full_Status(uint32_t Instance, uint8_t *Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Get_Full_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Get_Full_Status(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO decimation (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Function Motion sensor function. Could be:
 *         - MOTION_GYRO or MOTION_ACCELERO for instance IKS01A2_LSM6DSL_0
 * @param  Decimation FIFO decimation
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Decimation(uint32_t Instance, uint32_t Function, uint8_t Decimation)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM6DSL_FIFO_ACC_Set_Decimation(MotionCompObj[Instance], Decimation) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (LSM6DSL_FIFO_GYRO_Set_Decimation(MotionCompObj[Instance], Decimation) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (ISM330DLC_FIFO_ACC_Set_Decimation(MotionCompObj[Instance], Decimation) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (ISM330DLC_FIFO_GYRO_Set_Decimation(MotionCompObj[Instance], Decimation) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO ODR value (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Odr FIFO ODR value
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_ODR_Value(uint32_t Instance, float Odr)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Set_ODR_Value(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Set_ODR_Value(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO full interrupt on INT1 pin (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Status FIFO full interrupt on INT1 pin
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_INT1_FIFO_Full(uint32_t Instance, uint8_t Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Set_INT1_FIFO_Full(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Set_INT1_FIFO_Full(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO full interrupt on INT2 pin (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Status FIFO full interrupt on INT1 pin
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_INT2_FIFO_Full(uint32_t Instance, uint8_t Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Set_INT2_FIFO_Full(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO watermark level (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Watermark FIFO watermark level
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Watermark_Level(uint32_t Instance, uint16_t Watermark)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Set_Watermark_Level(MotionCompObj[Instance], Watermark) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Set_Watermark_Level(MotionCompObj[Instance], Watermark) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO stop on watermark (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Status FIFO stop on watermark status
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Stop_On_Fth(uint32_t Instance, uint8_t Status)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Set_Stop_On_Fth(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Set_Stop_On_Fth(MotionCompObj[Instance], Status) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set FIFO mode (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Mode FIFO mode
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Mode(uint32_t Instance, uint8_t Mode)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Set_Mode(MotionCompObj[Instance], Mode) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Set_Mode(MotionCompObj[Instance], Mode) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get FIFO pattern (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Pattern FIFO pattern
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Pattern(uint32_t Instance, uint16_t *Pattern)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if (LSM6DSL_FIFO_Get_Pattern(MotionCompObj[Instance], Pattern) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if (ISM330DLC_FIFO_Get_Pattern(MotionCompObj[Instance], Pattern) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get FIFO single axis data (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Function Motion sensor function. Could be:
 *         - MOTION_GYRO or MOTION_ACCELERO for instance IKS01A2_LSM6DSL_0
 * @param  Data FIFO single axis data
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Axis(uint32_t Instance, uint32_t Function, int32_t *Data)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM6DSL_FIFO_ACC_Get_Axis(MotionCompObj[Instance], Data) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (LSM6DSL_FIFO_GYRO_Get_Axis(MotionCompObj[Instance], Data) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (ISM330DLC_FIFO_ACC_Get_Axis(MotionCompObj[Instance], Data) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (ISM330DLC_FIFO_GYRO_Get_Axis(MotionCompObj[Instance], Data) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Get FIFO single axis data RAW
 * @param  Instance the device instance
 * @param  Function Motion sensor function. Could be:
 *         - MOTION_GYRO or MOTION_ACCELERO for instance IDP005_LSM6DSL_0
 * @param  Data FIFO single axis data
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Data_Word(uint32_t Instance, uint32_t Function, int16_t *Data)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (ISM330DLC_FIFO_Get_Data_Word(MotionCompObj[Instance], Data) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (ISM330DLC_FIFO_Get_Data_Word(MotionCompObj[Instance], Data) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

/**
 * @brief  Set accelero self-test (available only for LSM6DSL sensor)
 * @param  Instance the device instance
 * @param  Function Motion sensor function. Could be:
 *         - MOTION_GYRO or MOTION_ACCELERO for instance IKS01A2_LSM6DSL_0
 * @param  Mode self-test mode (0: disable self-test, 1: positive self-test, 2: negative self-test)
 * @retval BSP status
 */
int32_t IKS01A2_MOTION_SENSOR_Set_SelfTest(uint32_t Instance, uint32_t Function, uint8_t Mode)
{
  int32_t ret;

  switch (Instance)
  {
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
    case IKS01A2_LSM6DSL_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (LSM6DSL_ACC_Set_SelfTest(MotionCompObj[Instance], Mode) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (LSM6DSL_GYRO_Set_SelfTest(MotionCompObj[Instance], Mode) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      break;
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
    case IKS01A2_ISM330DLC_0:
      if ((Function & MOTION_ACCELERO) == MOTION_ACCELERO)
      {
        if (ISM330DLC_ACC_Set_SelfTest(MotionCompObj[Instance], Mode) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else if ((Function & MOTION_GYRO) == MOTION_GYRO)
      {
        if (ISM330DLC_GYRO_Set_SelfTest(MotionCompObj[Instance], Mode) != BSP_ERROR_NONE)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
      else
      {
        ret = BSP_ERROR_WRONG_PARAM;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
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
