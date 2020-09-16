/*
 * lsm303agr_rosService.cpp
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#include <ros/lsm6dsl_rosService.h>

LSM6DSL_rosService * LSM6DSL_rosService::LSM6DSL_Instance = NULL;


LSM6DSL_rosService * LSM6DSL_rosService::getInstance()
{
	if(!LSM6DSL_Instance)
	{
		LSM6DSL_Instance = new LSM6DSL_rosService();
	}

	return LSM6DSL_Instance;
}

LSM6DSL_rosService::LSM6DSL_rosService(const char * LSM6DSL_imu_desc,
		sensor_msgs::Imu * LSM6DSL_imu_sensor) :
		LSM6DSL_imu_t_(LSM6DSL_imu_sensor),
		LSM6DSL_imu_publisher_t(LSM6DSL_imu_desc, LSM6DSL_imu_sensor)
{
	//	TODO Check if node handler has already been initialized
	//	and initialize node if not already done
	serialInstance = serialInstance->getInstance();
}


// Use delegating constructor.
LSM6DSL_rosService::LSM6DSL_rosService() : LSM6DSL_rosService("LSM6DSL_Imu",
		new sensor_msgs::Imu())
{

}

int32_t LSM6DSL_rosService::LSM6DSL_AdvertizeImu(void)
{
	if(NULL == serialInstance)
	{
		return diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	//	TODO : Compete the diagnostic by competing the error msg
	if(!serialInstance->nh.advertise(LSM6DSL_imu_publisher_t))
	{
		return (int32_t) diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	return diagnostic_msgs::DiagnosticStatus::OK;
}

int32_t LSM6DSL_rosService::LSM6DSL_ImuPublish(void)
{
	return (int32_t) this->LSM6DSL_imu_publisher_t.publish(LSM6DSL_imu_t_);
}
