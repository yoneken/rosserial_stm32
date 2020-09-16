/*
 * lsm303agr_rosService.cpp
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#include "lsm303agr_rosService.h"

LSM303AGR_rosService * LSM303AGR_rosService::LSM303AGR_Instance = NULL;


LSM303AGR_rosService * LSM303AGR_rosService::getInstance()
{
	if(!LSM303AGR_Instance)
	{
		LSM303AGR_Instance = new LSM303AGR_rosService();
	}

	return LSM303AGR_Instance;
}

LSM303AGR_rosService::LSM303AGR_rosService(const char * LSM303AGR_mag_desc,
		sensor_msgs::MagneticField * LSM303AGR_mag_sensor,
		const char * LSM303AGR_accel_desc,
		geometry_msgs::AccelStamped * LSM303AGR_accel_sensor) :
		LSM303AGR_magnetic_field_t_(LSM303AGR_mag_sensor),
		LSM303AGR_magnetic_field_publisher_t(LSM303AGR_mag_desc, LSM303AGR_mag_sensor),
		LSM303AGR_acceleration_t_(LSM303AGR_accel_sensor),
		LSM303AGR_acceleration_publisher_t(LSM303AGR_accel_desc, LSM303AGR_accel_sensor)
{
	//	TODO Check if node handler has already been initialized
	//	and initialize node if not already done
	serialInstance = serialInstance->getInstance();
}


// Use delegating constructor.
LSM303AGR_rosService::LSM303AGR_rosService() : LSM303AGR_rosService("LSM303AGR_Magnetic_Field",
		new sensor_msgs::MagneticField(),
		"LSM303AGR_Acceleration",
		new geometry_msgs::AccelStamped())
{

}

int32_t LSM303AGR_rosService::LSM303AGR_AdvertizeMagneticField(void)
{
	if(NULL == serialInstance)
	{
		return diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	//	TODO : Compete the diagnostic by competing the error msg
	if(!serialInstance->nh.advertise(LSM303AGR_magnetic_field_publisher_t))
	{
		return (int32_t) diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	return diagnostic_msgs::DiagnosticStatus::OK;
}

int32_t LSM303AGR_rosService::LSM303AGR_AdvertizeAcceleration(void)
{
	if(NULL == serialInstance)
	{
		return diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	//	TODO : Compete the diagnostic by competing the error msg
	if(!serialInstance->nh.advertise(LSM303AGR_acceleration_publisher_t))
	{
		return (int32_t) diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	return diagnostic_msgs::DiagnosticStatus::OK;
}

int32_t LSM303AGR_rosService::LSM303AGR_MagneticFieldPublish(void)
{
	return (int32_t) this->LSM303AGR_magnetic_field_publisher_t.publish(LSM303AGR_magnetic_field_t_);
}

int32_t LSM303AGR_rosService::LSM303AGR_AccelerationPublish(void)
{
	return (int32_t) this->LSM303AGR_acceleration_publisher_t.publish(LSM303AGR_acceleration_t_);
}
