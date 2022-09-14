/*
 * mainpp.cpp
 *
 *  Created on: Apr 12, 2020
 *      Author: fofolevrai
 */
#include "main.h"
#include <ros.h>
#include <std_msgs/UInt16.h>
#include "mainpp.h"

ros::NodeHandle nh;

ros::Subscriber<std_msgs::UInt16> tim_channel1_led_subcriber("channel1_led", &tim_channel1_led_callBack);
ros::Subscriber<std_msgs::UInt16> tim_channel2_led_subcriber("channel2_led", &tim_channel2_led_callBack);
ros::Subscriber<std_msgs::UInt16> tim_channel3_led_subcriber("channel3_led", &tim_channel3_led_callBack);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  nh.getHardware()->reset_rbuf();
}


void tim_channel1_led_callBack(const std_msgs::UInt16& msg)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, msg.data);
}

void tim_channel2_led_callBack(const std_msgs::UInt16& msg)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, msg.data);
}

void tim_channel3_led_callBack(const std_msgs::UInt16& msg)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, msg.data);
}

void setup(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 5000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 7500);

	 nh.initNode();
	 nh.subscribe(tim_channel1_led_subcriber);
	 nh.subscribe(tim_channel2_led_subcriber);
	 nh.subscribe(tim_channel3_led_subcriber);
}

void loop(void)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	nh.spinOnce();

	HAL_Delay(1000);
}
