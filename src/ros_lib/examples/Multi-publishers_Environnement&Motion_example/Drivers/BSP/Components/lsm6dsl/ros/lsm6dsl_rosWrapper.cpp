/*
 * LSM6DSL_rosWrapper.cpp
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#include "ros/lsm6dsl_rosService.h"
#include "ros/lsm6dsl_rosWrapper.h"


static C_LSM6DSL_rosService_t * rosLSM6DSLInstance = NULL;

/**
 * Explicit constructor
 */
uint32_t new_C_LSM6DSL_rosService(void)
{
	if(NULL != (rosLSM6DSLInstance = reinterpret_cast<C_LSM6DSL_rosService_t*>(LSM6DSL_rosService::getInstance())))
	{
		return LSM6DSL_OK;
	}

	return LSM6DSL_ERROR;
}
/**
 * Explicit destructor
 */
void del_C_LSM6DSL_rosService(void)
{
	delete reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance);

	rosLSM6DSLInstance = NULL;
}


/**
 * @brief  Advertize HTS221 sensor capabilities to ROS master
 * @param  C_HTS221_rosService the ROSserial object reference
 * @param  Capabilities pointer to HTS221 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSL_RosAdvertize(LSM6DSL_Capabilities_t *Capabilities)
{
	static bool isAdvertized = false;

	if((!isAdvertized) && (Capabilities->Gyro) && (Capabilities->Acc))
	{
		reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_AdvertizeImu();
		isAdvertized = true;
	}

	return LSM6DSL_OK;
}

LSM6DSL_Axes_t LSM6DSL_RosGetAcceleration(LSM6DSL_Object_t *pObj)
{
	LSM6DSL_Axes_t l_LSM6DSL_axes_t = (const LSM6DSL_Axes_t) {0};

	if(pObj->acc_is_enabled)
	{
		l_LSM6DSL_axes_t.x = (int32_t) reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->linear_acceleration.x;
		l_LSM6DSL_axes_t.y = (int32_t) reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->linear_acceleration.y;
		l_LSM6DSL_axes_t.z = (int32_t) reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->linear_acceleration.z;
	}

	return l_LSM6DSL_axes_t;
}

void LSM6DSL_RosSetAcceleration(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *Acceleration)
{
	if(pObj->acc_is_enabled)
	{
		reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->linear_acceleration.x = (double) Acceleration->x;
		reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->linear_acceleration.y = (double) Acceleration->y;
		reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->linear_acceleration.z = (double) Acceleration->z;
	}
}

LSM6DSL_Axes_t LSM6DSL_RosGetGyroscope(LSM6DSL_Object_t *pObj)
{
	LSM6DSL_Axes_t AngularRate = (const LSM6DSL_Axes_t) {0};

	if(pObj->gyro_is_enabled)
	{
		AngularRate.x = reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->angular_velocity.x;
		AngularRate.y = reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->angular_velocity.y;
		AngularRate.z = reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->angular_velocity.z;
	}

	return AngularRate;
}

void LSM6DSL_RosSetGyroscope(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *AngularRate)
{
	if(pObj->gyro_is_enabled)
	{
		reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->angular_velocity.x = (double) AngularRate->x;
		reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->angular_velocity.y = (double) AngularRate->y;
		reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_imu_t()->angular_velocity.z = (double) AngularRate->z;
	}
}

int32_t LSM6DSL_RosImuPublish(LSM6DSL_Object_t *pObj)
{
	if((pObj->acc_is_enabled) || (pObj->gyro_is_enabled))
	{
		return (int32_t) reinterpret_cast<LSM6DSL_rosService*>(rosLSM6DSLInstance)->LSM6DSL_ImuPublish();
	}

	return LSM6DSL_ERROR;
}
