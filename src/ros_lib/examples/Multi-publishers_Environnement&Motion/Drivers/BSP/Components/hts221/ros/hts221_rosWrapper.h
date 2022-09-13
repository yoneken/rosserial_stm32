/*
 * hts221_rosWrapper.h
 *
 *  Created on: Jun 30, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_HTS221_HTS221_ROSWRAPPER_H_
#define BSP_COMPONENTS_HTS221_HTS221_ROSWRAPPER_H_

#include <stdint.h>
#include "hts221.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * C Interface for ROSserial class
 */
typedef struct C_HTS221_rosService_t C_HTS221_rosService_t;

/**
 * Explicit constructor
 */
uint32_t new_C_HTS221_rosService(void);
/**
 * Explicit destructor
 */
void del_C_HTS221_rosService(void);

/**
 * @brief  Advertize HTS221 sensor capabilities to ROS master
 * @param  C_HTS221_rosService the ROSserial object reference
 * @param  Capabilities pointer to HTS221 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HTS221_RosAdvertize(HTS221_Capabilities_t * Capabilities);

double HTS221_RosGetTemperature(HTS221_Object_t *pObj);

void HTS221_RosSetTemperature(HTS221_Object_t *pObj, double value);

double HTS221_RosGetHumidity(HTS221_Object_t *pObj);

void HTS221_RosSetHumidity(HTS221_Object_t *pObj, double value);

int32_t HTS221_RosTemperaturePublish(HTS221_Object_t *pObj);

int32_t HTS221_RosHumidityPublish(HTS221_Object_t *pObj);

int32_t HTS221_RosPublish(HTS221_Object_t *pObj, HTS221_Capabilities_t *Capabilities);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMPONENTS_HTS221_HTS221_ROSWRAPPER_H_ */
