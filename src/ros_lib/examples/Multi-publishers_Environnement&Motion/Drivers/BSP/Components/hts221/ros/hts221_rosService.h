/*
 * hts221_rosSerial.h
 *
 *  Created on: Jun 30, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_HTS221_HTS221_ROSSERIAL_H_
#define BSP_COMPONENTS_HTS221_HTS221_ROSSERIAL_H_

#include <ros.h>
#include <ros/msg.h>
#include "ROSserial.h"
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

class HTS221_rosService
{
private:
	static HTS221_rosService * HTS221_Instance;

	/*	Constructor for private usage */
	HTS221_rosService(const char * HTS221_temp_desc,
			sensor_msgs::Temperature * HTS221_temp_sensor,
			const char * HTS221_hum_desc,
			sensor_msgs::RelativeHumidity * HTS221_hum_sensor);

	ROSserial *serialInstance = NULL;

	/*	Sensor data variables & associated publishers/topics */
    sensor_msgs::Temperature *HTS221_air_temperature_t_;
    ros::Publisher HTS221_air_temperature_publisher_t;
    sensor_msgs::RelativeHumidity *HTS221_air_humidity_t_;
    ros::Publisher HTS221_air_humidity_publisher_t;

public:
    sensor_msgs::Temperature * HTS221_air_temperature_t()
    {
    	if(NULL != serialInstance)
    	{
        	HTS221_air_temperature_t_->header.stamp = serialInstance->nh.now();
    	}

    	return HTS221_air_temperature_t_;
    }
    const sensor_msgs::Temperature * HTS221_air_temperature_t() const { return HTS221_air_temperature_t_; }

    sensor_msgs::RelativeHumidity * HTS221_air_humidity_t()
    {
    	if(NULL != serialInstance)
    	{
    		HTS221_air_humidity_t_->header.stamp = serialInstance->nh.now();
    	}

    	return HTS221_air_humidity_t_;
    }
    const sensor_msgs::RelativeHumidity * HTS221_air_humidity_t() const { return HTS221_air_humidity_t_; }

    static HTS221_rosService * getInstance();

    /*	Public constructor */
    HTS221_rosService();

    int32_t HTS221_AdvertizeTemperature(void);

    int32_t HTS221_AdvertizeHumidity(void);

    int32_t HTS221_TemperaturePublish(void);

    int32_t HTS221_HumidityPublish(void);
};



#endif /* BSP_COMPONENTS_HTS221_HTS221_ROSSERIAL_H_ */
