/**
  ******************************************************************************
  * File Name          : stmicroelectronics_x-cube-mems1_7_1_0.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_X_CUBE_MEMS1_H
#define __APP_X_CUBE_MEMS1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
 /* Private includes ----------------------------------------------------------*/
 /* USER CODE BEGIN Includes */
 #include "rosSerial/ROSserial_Cwrapper.h"

 typedef struct displayFloatToInt_s {
   int8_t sign; /* 0 means positive, 1 means negative*/
   uint32_t  out_int;
   uint32_t  out_dec;
 } displayFloatToInt_t;

 /* USER CODE END Includes */
/* Exported defines ----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void MX_MEMS_Init();
void MX_MEMS_Process();

#ifdef __cplusplus
}
#endif

#endif /* __APP_X_CUBE_MEMS1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
