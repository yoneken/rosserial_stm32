/**
  ******************************************************************************
  * File Name          : stmicroelectronics_x-cube-mems1_7_1_0.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.7.1.0 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2020 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_x-cube-mems1.h"
#include "main.h"
#include <stdio.h>

#include "iks01a2_motion_sensors.h"
#include "iks01a2_env_sensors.h"
#include "math.h"

 /* USER CODE BEGIN User Include */
#include "rosSerial/ROSserial_Cwrapper.h"

 /* USER CODE END User Include */

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256
#define TERMINAL_VERBOSE	0	/* Verbose output to UART terminal ON/OFF. */

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t PushButtonDetected = 0;
static IKS01A2_MOTION_SENSOR_Capabilities_t MotionCapabilities[IKS01A2_MOTION_INSTANCES_NBR];
static IKS01A2_ENV_SENSOR_Capabilities_t EnvCapabilities[IKS01A2_ENV_INSTANCES_NBR];

/* Private function prototypes -----------------------------------------------*/
static void APP_X_MOTION_SENSOR_GetWhoAmI(uint32_t Instance, uint8_t * whoami);
static void APP_X_ENV_SENSOR_GetWhoAmI(uint32_t Instance, uint8_t * whoami);
static void Accelero_Sensor_Handler(uint32_t Instance, uint8_t * whoami);
static void Gyro_Sensor_Handler(uint32_t Instance, uint8_t * whoami);
static void Magneto_Sensor_Handler(uint32_t Instance, uint8_t * whoami);
static void Temp_Sensor_Handler(uint32_t Instance, uint8_t * whoami);
static void Hum_Sensor_Handler(uint32_t Instance, uint8_t * whoami);
static void Press_Sensor_Handler(uint32_t Instance, uint8_t * whoami);
static void MX_IKS01A2_DataLogTerminal_Init();
static void MX_IKS01A2_DataLogTerminal_Process();

void MX_MEMS_Init()
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_IKS01A2_DataLogTerminal_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_MEMS_Process()
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_IKS01A2_DataLogTerminal_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A2_DataLogTerminal_Init()
{
  int i;
#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
#endif

  for(i = 0; i < IKS01A2_MOTION_INSTANCES_NBR; i++)
  {
    IKS01A2_MOTION_SENSOR_GetCapabilities(i, &MotionCapabilities[i]);
    //AdvertizeMotionCapabilities_C_ROSserial(c_ROSserial, &MotionCapabilities[i]);

#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nMotion Sensor Instance %d capabilities: \r\n ACCELEROMETER: %d\r\n GYROSCOPE: %d\r\n MAGNETOMETER: %d\r\n LOW POWER: %d\r\n",
             i, MotionCapabilities[i].Acc, MotionCapabilities[i].Gyro, MotionCapabilities[i].Magneto, MotionCapabilities[i].LowPower);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].AccMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX ACC ODR: %d.%03d Hz, MAX ACC FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].AccMaxFS);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].GyroMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX GYRO ODR: %d.%03d Hz, MAX GYRO FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].GyroMaxFS);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].MagMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX MAG ODR: %d.%03d Hz, MAX MAG FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].MagMaxFS);
    printf("%s", dataOut);
#endif	/* TERMINAL_VERBOSE */
  }

#if USE_IKS01A2_ENV_SENSOR_HTS221_0
  IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
#endif

#if USE_IKS01A2_ENV_SENSOR_LPS22HB_0
  IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_TEMPERATURE | ENV_PRESSURE);
#endif

  for(i = 0; i < IKS01A2_ENV_INSTANCES_NBR; i++)
  {
    IKS01A2_ENV_SENSOR_GetCapabilities(i, &EnvCapabilities[i]);
    //AdvertizeEnvCapabilities_C_ROSserial(c_ROSserial, &EnvCapabilities[i]);

#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nEnvironmental Sensor Instance %d capabilities: \r\n TEMPERATURE: %d\r\n PRESSURE: %d\r\n HUMIDITY: %d\r\n LOW POWER: %d\r\n",
             i, EnvCapabilities[i].Temperature, EnvCapabilities[i].Pressure, EnvCapabilities[i].Humidity, EnvCapabilities[i].LowPower);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].TempMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX TEMP ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].PressMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX PRESS ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].HumMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX HUM ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int, (int)out_value_odr.out_dec);
    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
  }
}

/**
  * @brief  Process of the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A2_DataLogTerminal_Process(C_ROSserial_t * c_ROSserial)
{
  int i;
  uint8_t l_u8_whoami = 0xFF;

  for(i = 0; i < IKS01A2_MOTION_INSTANCES_NBR; i++)
  {
    if(MotionCapabilities[i].Acc)
    {
      Accelero_Sensor_Handler(i, &l_u8_whoami);
    }
    if(MotionCapabilities[i].Gyro)
    {
      Gyro_Sensor_Handler(i, &l_u8_whoami);
    }
    if(MotionCapabilities[i].Magneto)
    {
      Magneto_Sensor_Handler(i, &l_u8_whoami);
    }
  }

  for(i = 0; i < IKS01A2_ENV_INSTANCES_NBR; i++)
  {
    if(EnvCapabilities[i].Humidity)
    {
      Hum_Sensor_Handler(i, &l_u8_whoami);
    }
    if(EnvCapabilities[i].Temperature)
    {
      Temp_Sensor_Handler(i, &l_u8_whoami);
    }
    if(EnvCapabilities[i].Pressure)
    {
      Press_Sensor_Handler(i, &l_u8_whoami);
    }
  }
}

/**
  * @brief  Get the sensor ID for environement sensors
  * @param  Instance the device instance
  * @param  whoami the sensor ID variable
  * @retval None
  */
static void APP_X_MOTION_SENSOR_GetWhoAmI(uint32_t Instance, uint8_t * whoami)
{
	if (IKS01A2_MOTION_SENSOR_ReadID(Instance, whoami))
	{
#if   TERMINAL_VERBOSE
		snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
	    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
	}
	else
	{
#if TERMINAL_VERBOSE
		snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)*whoami);
	    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
	}
}

/**
  * @brief  Get the sensor ID for environement sensors
  * @param  Instance the device instance
  * @param  whoami the sensor ID variable
  * @retval None
  */
static void APP_X_ENV_SENSOR_GetWhoAmI(uint32_t Instance, uint8_t * whoami)
{
	if (IKS01A2_ENV_SENSOR_ReadID(Instance, whoami))
	{
#if   TERMINAL_VERBOSE
		snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
	    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
	}
	else
	{
#if TERMINAL_VERBOSE
		snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)*whoami);
	    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
	}
}

/**
  * @brief  Handles the accelerometer axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Accelero_Sensor_Handler(uint32_t Instance, uint8_t * whoami)
{
  IKS01A2_MOTION_SENSOR_Axes_t acceleration;

  //	Get the sensor ID whoami
  APP_X_MOTION_SENSOR_GetWhoAmI(Instance, whoami);

  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration))
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC[%d]: Error\r\n", (int)Instance);
#else

#endif /*	TERMINAL_VERBOSE */
  }
  else
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC_X[%d]: %d, ACC_Y[%d]: %d, ACC_Z[%d]: %d\r\n", (int)Instance,
             (int)acceleration.x, (int)Instance, (int)acceleration.y, (int)Instance, (int)acceleration.z);
#else

#endif /*	TERMINAL_VERBOSE */
  }
#if TERMINAL_VERBOSE
  printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_ACCELERO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, outValue, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)outValue->out_int,
               (int)outValue->out_dec);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_ACCELERO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d g\r\n", (int)Instance, (int)fullScale);
    }

    printf("%s", dataOut);
#endif /*	TERMINAL_VERBOSE */
}

/**
  * @brief  Handles the gyroscope axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Gyro_Sensor_Handler(uint32_t Instance, uint8_t * whoami)
{
  IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;

  //	Get the sensor ID whoami
  APP_X_MOTION_SENSOR_GetWhoAmI(Instance, whoami);

  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity))
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR[%d]: Error\r\n", (int)Instance);
#else
    //	RAISE ERROR

#endif /*	TERMINAL_VERBOSE */
  }
  else
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR_X[%d]: %d, GYR_Y[%d]: %d, GYR_Z[%d]: %d\r\n", (int)Instance,
             (int)angular_velocity.x, (int)Instance, (int)angular_velocity.y, (int)Instance, (int)angular_velocity.z);
#else
    //	Publish gyro data

#endif
  }

#if TERMINAL_VERBOSE
  printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_GYRO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, outValue, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)outValue->out_int,
               (int)outValue->out_dec);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_GYRO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d dps\r\n", (int)Instance, (int)fullScale);
    }

    printf("%s", dataOut);
#endif /*	TERMINAL_VERBOSE */
}

/**
  * @brief  Handles the magneto axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Magneto_Sensor_Handler(uint32_t Instance, uint8_t * whoami)
{
  IKS01A2_MOTION_SENSOR_Axes_t magnetic_field;

  //	Get the sensor ID whoami
  APP_X_MOTION_SENSOR_GetWhoAmI(Instance, whoami);

  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field))
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nMAG[%d]: Error\r\n", (int)Instance);
#else
    //	RAISE ERROR

#endif
  }
  else
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nMAG_X[%d]: %d, MAG_Y[%d]: %d, MAG_Z[%d]: %d\r\n", (int)Instance,
             (int)magnetic_field.x, (int)Instance, (int)magnetic_field.y, (int)Instance, (int)magnetic_field.z);
#else
    //	Publish magneto data

#endif
  }
#if TERMINAL_VERBOSE
  printf("%s", dataOut);


    if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_MAGNETO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, outValue, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)outValue->out_int,
               (int)outValue->out_dec);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_MAGNETO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d gauss\r\n", (int)Instance, (int)fullScale);
    }

    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
}

/**
  * @brief  Handles the temperature data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Temp_Sensor_Handler(uint32_t Instance, uint8_t * whoami)
{
  float temperature;

  //	Get the sensor ID whoami
  APP_X_ENV_SENSOR_GetWhoAmI(Instance, whoami);

  //	Get temperature from sensor
  if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature))
  {
#if TERMINAL_VERBOSE
	  //	RAISE ERROR
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nTemp[%d]: Error\r\n", (int)Instance);
#else
    //	Raise error

#endif /* TERMINAL_VERBOSE */
  }
  else
  {
#if TERMINAL_VERBOSE
	    floatToInt(temperature, outValue, 2);
	    snprintf(dataOut, MAX_BUF_SIZE, "\r\nTemp[%d]: %c%d.%02d degC\r\n", (int)Instance, ((outValue->sign) ? '-' : '+'), (int)outValue->out_int,
	             (int)outValue->out_dec);
#else
	  //	Set LPS22HB temperature
      //set_LPS22HB_temperature(c_ROSserial, (double) temperature);
#endif /* TERMINAL_VERBOSE */
  }

#if TERMINAL_VERBOSE
  printf("%s", dataOut);

    if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_TEMPERATURE, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, outValue, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)outValue->out_int,
               (int)outValue->out_dec);
    }

    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
}

/**
  * @brief  Handles the pressure sensor data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Press_Sensor_Handler(uint32_t Instance, uint8_t * whoami)
{
  float pressure;

  //	Get the sensor ID whoami
  APP_X_ENV_SENSOR_GetWhoAmI(Instance, whoami);

  if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure))
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress[%d]: Error\r\n", (int)Instance);
#else
    //	RAISE ERROR

#endif /* TERMINAL_VERBOSE */
  }
  else
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress[%d]: %d.%02d hPa\r\n", (int)Instance, (int)outValue->out_int,
             (int)outValue->out_dec);
#else
    //set_LPS22HB_pressure(c_ROSserial, (double) pressure);
#endif /* TERMINAL_VERBOSE */
  }

#if TERMINAL_VERBOSE
  printf("%s", dataOut);

    if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_PRESSURE, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, outValue, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)outValue->out_int,
               (int)outValue->out_dec);
    }

    printf("%s", dataOut);
#endif /* TERMINAL_VERBOSE */
}

/**
  * @brief  Handles the humidity data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Hum_Sensor_Handler(uint32_t Instance, uint8_t * whoami)
{
  float humidity;

  //	Get the sensor ID whoami
  APP_X_ENV_SENSOR_GetWhoAmI(Instance, whoami);

  if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity))
  {
#if TERMINAL_VERBOSE
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nHum[%d]: Error\r\n", (int)Instance);
#else
    //	RAISE ERROR
#endif /*	TERMINAL_VERBOSE */
  }
  else
  {
#if TERMINAL_VERBOSE
    floatToInt(humidity, outValue, 2);
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nHum[%d]: %d.%02d %%\r\n", (int)Instance, (int)outValue->out_int,
             (int)outValue->out_dec);
#else
    //set_HTS221_humidity(c_ROSserial, (double) humidity);
#endif	/* TERMINAL_VERBOSE */
  }

#if TERMINAL_VERBOSE

  printf("%s", dataOut);

    if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_HUMIDITY, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, outValue, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)outValue->out_int,
               (int)outValue->out_dec);
    }

    printf("%s", dataOut);
#endif 	/* TERMINAL_VERBOSE */
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
