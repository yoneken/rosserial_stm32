/*
 * lps22hb_rosWrapper.h
 *
 *  Created on: Jul 19, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_LPS22HB_ROS_LPS22HB_ROSWRAPPER_H_
#define BSP_COMPONENTS_LPS22HB_ROS_LPS22HB_ROSWRAPPER_H_

#include <stdint.h>
#include "lps22hb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * C Interface for ROSserial class
 */
typedef struct C_LPS22HB_rosService_t C_LPS22HB_rosService_t;

/**
 * Explicit constructor
 */
uint32_t new_C_LPS22HB_rosService(void);
/**
 * Explicit destructor
 */
void del_C_LPS22HB_rosService(void);

/**
 * @brief  Advertize HTS221 sensor capabilities to ROS master
 * @param  C_HTS221_rosService the ROSserial object reference
 * @param  Capabilities pointer to HTS221 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_RosAdvertize(LPS22HB_Capabilities_t * Capabilities);

double LPS22HB_RosGetTemperature(LPS22HB_Object_t *pObj);

void LPS22HB_RosSetTemperature(LPS22HB_Object_t *pObj, double value);

double LPS22HB_RosGetPressure(LPS22HB_Object_t *pObj);

void LPS22HB_RosSetPressure(LPS22HB_Object_t *pObj, double value);

int32_t LPS22HB_RosTemperaturePublish(LPS22HB_Object_t *pObj);

int32_t LPS22HB_RosPressurePublish(LPS22HB_Object_t *pObj);

int32_t LPS22HB_RosPublish(LPS22HB_Object_t *pObj, LPS22HB_Capabilities_t *Capabilities);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMPONENTS_LPS22HB_ROS_LPS22HB_ROSWRAPPER_H_ */
