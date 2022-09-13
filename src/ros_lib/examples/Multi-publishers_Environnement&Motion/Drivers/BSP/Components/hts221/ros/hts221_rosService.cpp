/*
 * hts221_rosSerial.cpp
 *
 *  Created on: Jun 30, 2020
 *      Author: fofolevrai
 */
#include "hts221_rosService.h"

HTS221_rosService * HTS221_rosService::HTS221_Instance = NULL;

HTS221_rosService * HTS221_rosService::getInstance()
{
	if(!HTS221_Instance)
	{
		HTS221_Instance = new HTS221_rosService();
	}

	return HTS221_Instance;
}

HTS221_rosService::HTS221_rosService(const char * HTS221_temp_desc,
		sensor_msgs::Temperature * HTS221_temp_sensor,
		const char * HTS221_hum_desc,
		sensor_msgs::RelativeHumidity * HTS221_hum_sensor) :
		HTS221_air_temperature_t_(HTS221_temp_sensor),
		HTS221_air_temperature_publisher_t(HTS221_temp_desc, HTS221_temp_sensor),
		HTS221_air_humidity_t_(HTS221_hum_sensor),
		HTS221_air_humidity_publisher_t(HTS221_hum_desc, HTS221_hum_sensor)
{
	//	TODO Check if node handler has already been initialized
	//	and initialize node if not already done
	serialInstance = serialInstance->getInstance();
}


// Use delegating constructor.
HTS221_rosService::HTS221_rosService() : HTS221_rosService("HTS221_Air_Temperature",
		new sensor_msgs::Temperature(),
		"HTS221_Air_Humidity",
		new sensor_msgs::RelativeHumidity())
{

}

int32_t HTS221_rosService::HTS221_AdvertizeTemperature(void)
{
	if(NULL == serialInstance)
	{
		return diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	//	TODO : Compete the diagnostic by competing the error msg
	if(!serialInstance->nh.advertise(HTS221_air_temperature_publisher_t))
	{
		return (int32_t) diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	return diagnostic_msgs::DiagnosticStatus::OK;
}

int32_t HTS221_rosService::HTS221_AdvertizeHumidity(void)
{
	if(NULL == serialInstance)
	{
		return diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	//	TODO : Compete the diagnostic by competing the error msg
	if(!serialInstance->nh.advertise(HTS221_air_humidity_publisher_t))
	{
		return (int32_t) diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	return diagnostic_msgs::DiagnosticStatus::OK;
}

int32_t HTS221_rosService::HTS221_TemperaturePublish(void)
{
	return (int32_t) this->HTS221_air_temperature_publisher_t.publish(HTS221_air_temperature_t_);
}

int32_t HTS221_rosService::HTS221_HumidityPublish(void)
{
	return (int32_t) this->HTS221_air_humidity_publisher_t.publish(HTS221_air_humidity_t_);
}
