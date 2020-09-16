/*
 * mainpp.h
 *
 *  Created on: Apr 12, 2020
 *      Author: fofolevrai
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

extern void setup(void);
extern void loop(void);

typedef union
{
	struct
	{
		uint8_t pressureData[3];
		uint8_t temperatureData[2];
	};
	uint8_t rawData[5];
} Lps322hb_out_data_t;

#endif /* INC_MAINPP_H_ */
