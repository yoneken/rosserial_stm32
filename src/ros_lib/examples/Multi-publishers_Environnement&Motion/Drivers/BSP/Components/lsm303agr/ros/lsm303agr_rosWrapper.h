/*
 * lsm303agr_rosWrapper.h
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_LSM303AGR_ROS_LSM303AGR_ROSWRAPPER_H_
#define BSP_COMPONENTS_LSM303AGR_ROS_LSM303AGR_ROSWRAPPER_H_

#include <stdint.h>
#include "lsm303agr.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * C Interface for ROSserial class
 */
typedef struct C_LSM303AGR_rosService_t C_LSM303AGR_rosService_t;

/**
 * Explicit constructor
 */
uint32_t new_C_LSM303AGR_rosService(void);
/**
 * Explicit destructor
 */
void del_C_LSM303AGR_rosService(void);

/**
 * @brief  Advertize LSM303AGR sensor capabilities to ROS master
 * @param  C_LSM303AGR_rosService the ROSserial object reference
 * @param  Capabilities pointer to LSM303AGR sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_RosAdvertize(LSM303AGR_Capabilities_t * Capabilities);

LSM303AGR_Axes_t LSM303AGR_RosGetMagneticField(LSM303AGR_MAG_Object_t *pObj);

void LSM303AGR_RosSetMagneticField(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Axes_t * Acceleration);

LSM303AGR_Axes_t LSM303AGR_RosGetAcceleration(LSM303AGR_ACC_Object_t *pObj);

void LSM303AGR_RosSetAcceleration(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Axes_t * Acceleration);

int32_t LSM303AGR_RosMagneticFieldPublish(LSM303AGR_MAG_Object_t *pObj);

int32_t LSM303AGR_RosAccelerationPublish(LSM303AGR_ACC_Object_t *pObj);

int32_t LSM303AGR_RosPublish(LSM303AGR_MAG_Object_t *pMagObj, LSM303AGR_ACC_Object_t *pAccObj);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMPONENTS_LSM303AGR_ROS_LSM303AGR_ROSWRAPPER_H_ */
