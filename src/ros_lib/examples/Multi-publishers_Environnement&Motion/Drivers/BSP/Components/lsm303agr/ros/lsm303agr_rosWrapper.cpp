/*
 * lsm303agr_rosWrapper.cpp
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#include "ros/lsm303agr_rosService.h"
#include "ros/lsm303agr_rosWrapper.h"


static C_LSM303AGR_rosService_t * rosLSM303AGRInstance = NULL;

/**
 * Explicit constructor
 */
uint32_t new_C_LSM303AGR_rosService(void)
{
	if(NULL != (rosLSM303AGRInstance = reinterpret_cast<C_LSM303AGR_rosService_t*>(LSM303AGR_rosService::getInstance())))
	{
		return LSM303AGR_OK;
	}

	return LSM303AGR_ERROR;
}
/**
 * Explicit destructor
 */
void del_C_LSM303AGR_rosService(void)
{
	delete reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance);

	rosLSM303AGRInstance = NULL;
}


/**
 * @brief  Advertize HTS221 sensor capabilities to ROS master
 * @param  C_HTS221_rosService the ROSserial object reference
 * @param  Capabilities pointer to HTS221 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM303AGR_RosAdvertize(LSM303AGR_Capabilities_t *Capabilities)
{
	if(Capabilities->Magneto)
	{
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_AdvertizeMagneticField();
	}
	if(Capabilities->Acc)
	{
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_AdvertizeAcceleration();
	}

	return LSM303AGR_OK;
}

LSM303AGR_Axes_t LSM303AGR_RosGetMagneticField(LSM303AGR_MAG_Object_t *pObj)
{
	LSM303AGR_Axes_t l_lsm303agr_axes_t = (const LSM303AGR_Axes_t) {0};

	if(pObj->mag_is_enabled)
	{
		l_lsm303agr_axes_t.x = (int32_t) reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_magnetic_field_t()->magnetic_field.x;
		l_lsm303agr_axes_t.y = (int32_t) reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_magnetic_field_t()->magnetic_field.y;
		l_lsm303agr_axes_t.z = (int32_t) reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_magnetic_field_t()->magnetic_field.z;
	}

	return l_lsm303agr_axes_t;
}

void LSM303AGR_RosSetMagneticField(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Axes_t *Acceleration)
{
	if(pObj->mag_is_enabled)
	{
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_magnetic_field_t()->magnetic_field.x = (double) Acceleration->x;
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_magnetic_field_t()->magnetic_field.y = (double) Acceleration->y;
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_magnetic_field_t()->magnetic_field.z = (double) Acceleration->z;
	}
}

LSM303AGR_Axes_t LSM303AGR_RosGetAcceleration(LSM303AGR_ACC_Object_t *pObj)
{
	geometry_msgs::AccelStamped * l_ACC_AccelStamped_t;

	LSM303AGR_Axes_t l_ACC_LSM303AGR_Axes_t = (const LSM303AGR_Axes_t) {0};

	if(pObj->acc_is_enabled)
	{
		l_ACC_AccelStamped_t = reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_acceleration_t();

		l_ACC_LSM303AGR_Axes_t.x = l_ACC_AccelStamped_t->accel.linear.x;
		l_ACC_LSM303AGR_Axes_t.y = l_ACC_AccelStamped_t->accel.linear.y;
		l_ACC_LSM303AGR_Axes_t.z = l_ACC_AccelStamped_t->accel.linear.z;
	}

	return l_ACC_LSM303AGR_Axes_t;
}

void LSM303AGR_RosSetAcceleration(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Axes_t *Acceleration)
{
	if(pObj->acc_is_enabled)
	{
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_acceleration_t()->accel.linear.x = (double) Acceleration->x;
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_acceleration_t()->accel.linear.y = (double) Acceleration->y;
		reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_acceleration_t()->accel.linear.z = (double) Acceleration->z;
	}
}

int32_t LSM303AGR_RosMagneticFieldPublish(LSM303AGR_MAG_Object_t *pObj)
{
	if(pObj->mag_is_enabled)
	{
		return (int32_t) reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_MagneticFieldPublish();
	}

	return LSM303AGR_ERROR;
}

int32_t LSM303AGR_RosAccelerationPublish(LSM303AGR_ACC_Object_t *pObj)
{
	if(pObj->acc_is_enabled)
	{
		return (int32_t) reinterpret_cast<LSM303AGR_rosService*>(rosLSM303AGRInstance)->LSM303AGR_AccelerationPublish();
	}

	return LSM303AGR_ERROR;
}

int32_t LSM303AGR_RosPublish(LSM303AGR_MAG_Object_t *pMagObj, LSM303AGR_ACC_Object_t *pAccObj)
{
	return LSM303AGR_RosMagneticFieldPublish(pMagObj) & LSM303AGR_RosAccelerationPublish(pAccObj);
}
