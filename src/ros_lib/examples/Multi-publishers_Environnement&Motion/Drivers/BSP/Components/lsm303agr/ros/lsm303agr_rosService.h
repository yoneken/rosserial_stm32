/*
 * lsm303agr_rosService.h
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_LSM303AGR_ROS_LSM303AGR_ROSSERVICE_H_
#define BSP_COMPONENTS_LSM303AGR_ROS_LSM303AGR_ROSSERVICE_H_

#include <ros.h>
#include <ros/msg.h>
#include "ROSserial.h"
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/AccelStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>


class LSM303AGR_rosService
{
private:
	static LSM303AGR_rosService * LSM303AGR_Instance;

	/*	Constructor for private usage */
	LSM303AGR_rosService(const char * LSM303AGR_mag_desc,
			sensor_msgs::MagneticField * LSM303AGR_mag_sensor,
			const char * LSM303AGR_accel_desc,
			geometry_msgs::AccelStamped * LSM303AGR_accel_sensor);

	ROSserial *serialInstance = NULL;

	/*	Sensor data variables & associated publishers/topics */
    sensor_msgs::MagneticField * LSM303AGR_magnetic_field_t_;
    ros::Publisher LSM303AGR_magnetic_field_publisher_t;
    geometry_msgs::AccelStamped * LSM303AGR_acceleration_t_;
    ros::Publisher LSM303AGR_acceleration_publisher_t;

public:
    sensor_msgs::MagneticField * LSM303AGR_magnetic_field_t()
    {
    	if(NULL != serialInstance)
    	{
        	LSM303AGR_magnetic_field_t_->header.stamp = serialInstance->nh.now();
    	}

    	return LSM303AGR_magnetic_field_t_;
    }
    const sensor_msgs::MagneticField * LSM303AGR_magnetic_field_t() const { return LSM303AGR_magnetic_field_t_; }

    geometry_msgs::AccelStamped * LSM303AGR_acceleration_t()
    {
    	if(NULL != serialInstance)
    	{
    		LSM303AGR_acceleration_t_->header.stamp = serialInstance->nh.now();
    	}

    	return LSM303AGR_acceleration_t_;
    }
    const geometry_msgs::AccelStamped * LSM303AGR_acceleration_t() const { return LSM303AGR_acceleration_t_; }

    static LSM303AGR_rosService * getInstance();

    /*	Public constructor */
    LSM303AGR_rosService();

    int32_t LSM303AGR_AdvertizeMagneticField(void);

    int32_t LSM303AGR_AdvertizeAcceleration(void);

    int32_t LSM303AGR_MagneticFieldPublish(void);

    int32_t LSM303AGR_AccelerationPublish(void);
};


#endif /* BSP_COMPONENTS_LSM303AGR_ROS_LSM303AGR_ROSSERVICE_H_ */
