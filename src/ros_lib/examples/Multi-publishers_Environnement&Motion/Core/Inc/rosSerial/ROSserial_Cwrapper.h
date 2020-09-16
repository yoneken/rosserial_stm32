/*
 * ROSserial_Cwrapper.h
 *
 *  Created on: May 9, 2020
 *      Author: fofolevrai
 */

#ifndef INC_ROSSERIAL_CWRAPPER_H_
#define INC_ROSSERIAL_CWRAPPER_H_

#include "iks01a2_motion_sensors.h"
#include "iks01a2_env_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * C Interface for ROSserial class
 */
typedef struct C_ROSserial_t C_ROSserial_t;

/**
 * Explicit constructor
 */
C_ROSserial_t * new_C_ROSserial(void);
/**
 * Explicit destructor
 */
void del_C_ROSserial(C_ROSserial_t * c_ROSserial);

uint8_t Reset_C_ROSserial(C_ROSserial_t * c_ROSserial);

uint8_t Flush_C_ROSserial(C_ROSserial_t * c_ROSserial);

/**
 * Advertize client of motion sensors capabilities
 */
uint8_t AdvertizeMotionCapabilities_C_ROSserial(C_ROSserial_t * c_ROSserial, IKS01A2_MOTION_SENSOR_Capabilities_t *cap);

void SpinOnce_C_ROSserial(C_ROSserial_t * c_ROSserial);

#ifdef __cplusplus
}
#endif

#endif /* INC_ROSSERIAL_CWRAPPER_H_ */
