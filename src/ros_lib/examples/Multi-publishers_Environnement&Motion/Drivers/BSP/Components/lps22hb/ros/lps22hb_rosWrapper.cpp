/*
 * lps22hb_rosWrapper.cpp
 *
 *  Created on: Jul 19, 2020
 *      Author: fofolevrai
 */

#include "ros/lps22hb_rosService.h"
#include "ros/lps22hb_rosWrapper.h"

static C_LPS22HB_rosService_t * rosLPS22HBInstance = NULL;

/**
 * Explicit constructor
 */
uint32_t new_C_LPS22HB_rosService(void)
{
	if(NULL != (rosLPS22HBInstance = reinterpret_cast<C_LPS22HB_rosService_t*>(LPS22HB_rosService::getInstance())))
	{
		return LPS22HB_OK;
	}

	return LPS22HB_ERROR;
}
/**
 * Explicit destructor
 */
void del_C_LPS22HB_rosService(void)
{
	delete reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance);

	rosLPS22HBInstance = NULL;
}


/**
 * @brief  Advertize HTS221 sensor capabilities to ROS master
 * @param  C_HTS221_rosService the ROSserial object reference
 * @param  Capabilities pointer to HTS221 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS22HB_RosAdvertize(LPS22HB_Capabilities_t *Capabilities)
{
	if(Capabilities->Temperature)
	{
		reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_AdvertizeTemperature();
	}
	if(Capabilities->Pressure)
	{
		reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_AdvertizePressure();
	}

	return LPS22HB_OK;
}

double LPS22HB_RosGetTemperature(LPS22HB_Object_t *pObj)
{
	if(pObj->temp_is_enabled)
	{
		return reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_air_temperature_t()->temperature;
	}

	return LPS22HB_ERROR;
}

void LPS22HB_RosSetTemperature(LPS22HB_Object_t *pObj, double value)
{
	if(pObj->temp_is_enabled)
	{
		reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_air_temperature_t()->temperature =  value;
	}
}

double LPS22HB_RosGetPressure(LPS22HB_Object_t *pObj)
{
	if(pObj->press_is_enabled)
	{
		return reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_air_pressure_t()->fluid_pressure;
	}

	return LPS22HB_ERROR;
}

void LPS22HB_RosSetPressure(LPS22HB_Object_t *pObj, double value)
{
	if(pObj->press_is_enabled)
	{
		reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_air_pressure_t()->fluid_pressure =  value;
	}
}

int32_t LPS22HB_RosTemperaturePublish(LPS22HB_Object_t *pObj)
{
	if(pObj->temp_is_enabled)
	{
		return (int32_t) reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_TemperaturePublish();
	}

	return LPS22HB_ERROR;
}

int32_t LPS22HB_RosPressurePublish(LPS22HB_Object_t *pObj)
{
	if(pObj->press_is_enabled)
	{
		return (int32_t) reinterpret_cast<LPS22HB_rosService*>(rosLPS22HBInstance)->LPS22HB_PressurePublish();
	}

	return LPS22HB_ERROR;
}

int32_t LPS22HB_RosPublish(LPS22HB_Object_t *pObj)
{
	return LPS22HB_RosTemperaturePublish(pObj) & LPS22HB_RosPressurePublish(pObj);
}

