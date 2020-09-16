/*
 * hts221_rosWrapper.cpp
 *
 *  Created on: Jun 30, 2020
 *      Author: fofolevrai
 */
#include <stdint.h>
#include "ros/hts221_rosService.h"
#include "ros/hts221_rosWrapper.h"

static C_HTS221_rosService_t * rosHTS221Instance = NULL;
/**
 * Explicit constructor
 */
uint32_t new_C_HTS221_rosService(void)
{
	if(NULL != (rosHTS221Instance = reinterpret_cast<C_HTS221_rosService_t*>(HTS221_rosService::getInstance())))
	{
		return HTS221_OK;
	}

	return HTS221_ERROR;
}
/**
 * Explicit destructor
 */
void del_C_HTS221_rosService(void)
{
	delete reinterpret_cast<HTS221_rosService*>(rosHTS221Instance);

	rosHTS221Instance = NULL;
}


/**
 * @brief  Advertize HTS221 sensor capabilities to ROS master
 * @param  C_HTS221_rosService the ROSserial object reference
 * @param  Capabilities pointer to HTS221 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HTS221_RosAdvertize(HTS221_Capabilities_t *Capabilities)
{
	if(Capabilities->Temperature)
	{
		reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_AdvertizeTemperature();
	}
	if(Capabilities->Humidity)
	{
		reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_AdvertizeHumidity();
	}

	return HTS221_OK;
}

double HTS221_RosGetTemperature(HTS221_Object_t *pObj)
{
	if(pObj->temp_is_enabled)
	{
		return reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_air_temperature_t()->temperature;
	}

	return HTS221_ERROR;
}

void HTS221_RosSetTemperature(HTS221_Object_t *pObj, double value)
{
	if(pObj->temp_is_enabled)
	{
		reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_air_temperature_t()->temperature =  value;
	}
}

double HTS221_RosGetHumidity(HTS221_Object_t *pObj)
{
	if(pObj->hum_is_enabled)
	{
		return reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_air_humidity_t()->relative_humidity;
	}

	return HTS221_ERROR;
}

void HTS221_RosSetHumidity(HTS221_Object_t *pObj, double value)
{
	if(pObj->hum_is_enabled)
	{
		reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_air_humidity_t()->relative_humidity =  value;
	}
}

int32_t HTS221_RosTemperaturePublish(HTS221_Object_t *pObj)
{
	if(pObj->temp_is_enabled)
	{
		return (int32_t) reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_TemperaturePublish();
	}

	return HTS221_ERROR;
}

int32_t HTS221_RosHumidityPublish(HTS221_Object_t *pObj)
{
	if(pObj->hum_is_enabled)
	{
		return (int32_t) reinterpret_cast<HTS221_rosService*>(rosHTS221Instance)->HTS221_HumidityPublish();
	}

	return HTS221_ERROR;
}

int32_t HTS221_RosPublish(HTS221_Object_t *pObj)
{
	return HTS221_RosTemperaturePublish(pObj) & HTS221_RosHumidityPublish(pObj);
}
