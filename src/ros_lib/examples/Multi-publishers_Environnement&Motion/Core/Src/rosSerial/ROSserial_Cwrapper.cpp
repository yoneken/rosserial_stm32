/*
 * ROSserial_Cwrapper.cpp
 *
 *  Created on: May 9, 2020
 *      Author: fofolevrai
 */
#include "rosSerial/ROSserial.h"
#include "ROSserial_Cwrapper.h"
#include "app_x-cube-mems1.h"

ROSserial * rosSerialInstance = NULL;

C_ROSserial_t * new_C_ROSserial(void)
{
	rosSerialInstance = rosSerialInstance->getInstance();

	return reinterpret_cast<C_ROSserial_t*>(rosSerialInstance);
}

void del_C_ROSserial(C_ROSserial_t * c_ROSserial)
{
	delete reinterpret_cast<ROSserial*>(c_ROSserial);
}

uint8_t Reset_C_ROSserial(C_ROSserial_t * c_ROSserial)
{
	if(NULL == c_ROSserial)
	{
		return false;
	}

	return (bool) reinterpret_cast<ROSserial*>(c_ROSserial)->ResetBuffer();
}

uint8_t Flush_C_ROSserial(C_ROSserial_t * c_ROSserial)
{
	if(NULL == c_ROSserial)
	{
		return false;
	}

	return (bool) reinterpret_cast<ROSserial*>(c_ROSserial)->FlushBuffer();
}

void SpinOnce_C_ROSserial(C_ROSserial_t * c_ROSserial)
{
	reinterpret_cast<ROSserial*>(c_ROSserial)->SpinOnce();
}


