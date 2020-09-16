/*
 * ROSserial.h
 *
 *  Created on: May 3, 2020
 *      Author: fofolevrai
 */
#ifndef INC_ROSSERIAL_H_
#define INC_ROSSERIAL_H_

#include <ros.h>
#include <ros/msg.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

class ROSserial
{
   private:
	//	Singleton pattern
	static ROSserial * instance;

    //  Class constructor
    ROSserial();

   public:
    //  Node handler
    ros::NodeHandle nh;

    static ROSserial * getInstance();

    bool FlushBuffer(void);

    bool ResetBuffer(void);

    void SpinOnce(void);
};

#endif /* INC_ROSSERIAL_H_ */
