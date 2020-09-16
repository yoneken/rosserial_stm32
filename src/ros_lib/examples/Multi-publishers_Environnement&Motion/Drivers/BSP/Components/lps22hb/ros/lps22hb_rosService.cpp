/*
 * lps22hb_rosService.cpp
 *
 *  Created on: Jul 19, 2020
 *      Author: fofolevrai
 */

#include "lps22hb_rosService.h"


LPS22HB_rosService * LPS22HB_rosService::LPS22HB_Instance = NULL;

LPS22HB_rosService * LPS22HB_rosService::getInstance()
{
	if(!LPS22HB_Instance)
	{
		LPS22HB_Instance = new LPS22HB_rosService();
	}

	return LPS22HB_Instance;
}

LPS22HB_rosService::LPS22HB_rosService(const char * LPS22HB_temp_desc,
		sensor_msgs::Temperature * LPS22HB_temp_sensor,
		const char * LPS22HB_pres_desc,
		sensor_msgs::FluidPressure * LPS22HB_pres_sensor) :
		LPS22HB_air_temperature_t_(LPS22HB_temp_sensor),
		LPS22HB_air_temperature_publisher_t(LPS22HB_temp_desc, LPS22HB_temp_sensor),
		LPS22HB_air_pressure_t_(LPS22HB_pres_sensor),
		LPS22HB_air_pressure_publisher_t(LPS22HB_pres_desc, LPS22HB_pres_sensor)
{
	//	TODO Check if node handler has already been initialized
	//	and initialize node if not already done
	serialInstance = serialInstance->getInstance();
}


// Use delegating constructor.
LPS22HB_rosService::LPS22HB_rosService() : LPS22HB_rosService("LPS22HB_Air_Temperature",
		new sensor_msgs::Temperature(),
		"LPS22HB_Air_Pressure",
		new sensor_msgs::FluidPressure())
{

}

int32_t LPS22HB_rosService::LPS22HB_AdvertizeTemperature(void)
{
	if(NULL == serialInstance)
	{
		return diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	//	TODO : Compete the diagnostic by competing the error msg
	if(!serialInstance->nh.advertise(LPS22HB_air_temperature_publisher_t))
	{
		return (int32_t) diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	return diagnostic_msgs::DiagnosticStatus::OK;
}

int32_t LPS22HB_rosService::LPS22HB_AdvertizePressure(void)
{
	if(NULL == serialInstance)
	{
		return diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	//	TODO : Compete the diagnostic by competing the error msg
	if(!serialInstance->nh.advertise(LPS22HB_air_pressure_publisher_t))
	{
		return (int32_t) diagnostic_msgs::DiagnosticStatus::ERROR;
	}

	return diagnostic_msgs::DiagnosticStatus::OK;
}

int32_t LPS22HB_rosService::LPS22HB_TemperaturePublish(void)
{
	return (int32_t) this->LPS22HB_air_temperature_publisher_t.publish(LPS22HB_air_temperature_t_);
}

int32_t LPS22HB_rosService::LPS22HB_PressurePublish(void)
{
	return (int32_t) this->LPS22HB_air_pressure_publisher_t.publish(LPS22HB_air_pressure_t_);
}



