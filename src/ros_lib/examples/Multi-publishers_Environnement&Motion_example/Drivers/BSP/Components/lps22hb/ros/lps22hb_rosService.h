/*
 * lps22hb_rosService.h
 *
 *  Created on: Jul 19, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_LPS22HB_ROS_LPS22HB_ROSSERVICE_H_
#define BSP_COMPONENTS_LPS22HB_ROS_LPS22HB_ROSSERVICE_H_

#include <ros.h>
#include <ros/msg.h>
#include "ROSserial.h"
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

class LPS22HB_rosService
{
private:
	static LPS22HB_rosService * LPS22HB_Instance;

	/*	Constructor for private usage */
	LPS22HB_rosService(const char * LPS22HB_temp_desc,
			sensor_msgs::Temperature * LPS22HB_temp_sensor,
			const char * LPS22HB_hum_desc,
			sensor_msgs::FluidPressure * LPS22HB_hum_sensor);

	ROSserial *serialInstance = NULL;

	/*	Sensor data variables & associated publishers/topics */
    sensor_msgs::Temperature *LPS22HB_air_temperature_t_;
    ros::Publisher LPS22HB_air_temperature_publisher_t;
    sensor_msgs::FluidPressure *LPS22HB_air_pressure_t_;
    ros::Publisher LPS22HB_air_pressure_publisher_t;

public:
    sensor_msgs::Temperature * LPS22HB_air_temperature_t()
    {
    	if(NULL != serialInstance)
    	{
        	LPS22HB_air_temperature_t_->header.stamp = serialInstance->nh.now();
    	}

    	return LPS22HB_air_temperature_t_;
    }
    const sensor_msgs::Temperature * LPS22HB_air_temperature_t() const { return LPS22HB_air_temperature_t_; }

    sensor_msgs::FluidPressure * LPS22HB_air_pressure_t()
    {
    	if(NULL != serialInstance)
    	{
    		LPS22HB_air_pressure_t_->header.stamp = serialInstance->nh.now();
    	}

    	return LPS22HB_air_pressure_t_;
    }
    const sensor_msgs::FluidPressure * LPS22HB_air_pressure_t() const { return LPS22HB_air_pressure_t_; }

    static LPS22HB_rosService * getInstance();

    /*	Public constructor */
    LPS22HB_rosService();

    int32_t LPS22HB_AdvertizeTemperature(void);

    int32_t LPS22HB_AdvertizePressure(void);

    int32_t LPS22HB_TemperaturePublish(void);

    int32_t LPS22HB_PressurePublish(void);
};

#endif /* BSP_COMPONENTS_LPS22HB_ROS_LPS22HB_ROSSERVICE_H_ */
