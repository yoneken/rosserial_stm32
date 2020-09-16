/*
 * mainpp.h
 *
 *  Created on: Apr 12, 2020
 *      Author: fofolevrai
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_


void tim_channel1_led_callBack(const std_msgs::UInt16& msg);
void tim_channel2_led_callBack(const std_msgs::UInt16& msg);
void tim_channel3_led_callBack(const std_msgs::UInt16& msg);
void tim_channel4_led_callBack(const std_msgs::UInt16& msg);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_MAINPP_H_ */
