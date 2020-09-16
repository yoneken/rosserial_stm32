/*
 * LSM6DSL_rosWrapper.h
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_LSM6DSL_ROS_LSM6DSL_ROSWRAPPER_H_
#define BSP_COMPONENTS_LSM6DSL_ROS_LSM6DSL_ROSWRAPPER_H_

#include <stdint.h>
#include "lsm6dsl.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * C Interface for ROSserial class
 */
typedef struct C_LSM6DSL_rosService_t C_LSM6DSL_rosService_t;

/**
 * Explicit constructor
 */
uint32_t new_C_LSM6DSL_rosService(void);
/**
 * Explicit destructor
 */
void del_C_LSM6DSL_rosService(void);

/**
 * @brief  Advertize LSM6DSL sensor capabilities to ROS master
 * @param  C_LSM6DSL_rosService the ROSserial object reference
 * @param  Capabilities pointer to LSM6DSL sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSL_RosAdvertize(LSM6DSL_Capabilities_t * Capabilities);

LSM6DSL_Axes_t LSM6DSL_RosGetAcceleration(LSM6DSL_Object_t *pObj);

void LSM6DSL_RosSetAcceleration(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t * Acceleration);

LSM6DSL_Axes_t LSM6DSL_RosGetGyroscope(LSM6DSL_Object_t *pObj);

void LSM6DSL_RosSetGyroscope(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t * AngularRate);

int32_t LSM6DSL_RosImuPublish(LSM6DSL_Object_t *pObj);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMPONENTS_LSM6DSL_ROS_LSM6DSL_ROSWRAPPER_H_ */
