/*
 * ROSserial.cpp
 *
 *  Created on: May 4, 2020
 *      Author: fofolevrai
 */
#include "rosSerial/ROSserial.h"

ROSserial * ROSserial::instance = NULL;

//	Singleton pattern design
ROSserial * ROSserial::getInstance()
{
	if(!instance)
	{
    	instance = new ROSserial();
    }

    return instance;
}

ROSserial::ROSserial()
{
   // Initialize ROS node
   this->nh.initNode();
}

bool ROSserial::FlushBuffer(void)
{
	nh.getHardware()->flush();

	return true;
}

bool ROSserial::ResetBuffer(void)
{
	nh.getHardware()->reset_rbuf();

	return true;
}

void ROSserial::SpinOnce(void)
{
	nh.spinOnce();
}
