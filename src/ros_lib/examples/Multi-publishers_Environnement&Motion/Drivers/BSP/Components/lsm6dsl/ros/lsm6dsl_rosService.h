/*
 * lsm6dsl_rosService.h
 *
 *  Created on: Jul 22, 2020
 *      Author: fofolevrai
 */

#ifndef BSP_COMPONENTS_LSM6DSL_ROS_LSM303AGR_ROSSERVICE_H_
#define BSP_COMPONENTS_LSM6DSL_ROS_LSM303AGR_ROSSERVICE_H_

#include <ros.h>
#include <ros/msg.h>
#include "ROSserial.h"
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticStatus.h>


class LSM6DSL_rosService
{
private:
	static LSM6DSL_rosService * LSM6DSL_Instance;

	/*	Constructor for private usage */
	LSM6DSL_rosService(const char * LSM6DSL_imu_desc,
			sensor_msgs::Imu * LSM6DSL_imu_sensor);

	ROSserial *serialInstance = NULL;

	/*	Sensor data variables & associated publishers/topics */
    sensor_msgs::Imu * LSM6DSL_imu_t_;
    ros::Publisher LSM6DSL_imu_publisher_t;

public:
    sensor_msgs::Imu * LSM6DSL_imu_t()
    {
    	if(NULL != serialInstance)
    	{
        	LSM6DSL_imu_t_->header.stamp = serialInstance->nh.now();
    	}

    	return LSM6DSL_imu_t_;
    }
    const sensor_msgs::Imu * LSM6DSL_imu_t() const { return LSM6DSL_imu_t_; }

    static LSM6DSL_rosService * getInstance();

    /*	Public constructor */
    LSM6DSL_rosService();

    int32_t LSM6DSL_AdvertizeImu(void);

    int32_t LSM6DSL_ImuPublish(void);
};


#endif /* BSP_COMPONENTS_LSM303AGR_ROS_LSM303AGR_ROSSERVICE_H_ */
