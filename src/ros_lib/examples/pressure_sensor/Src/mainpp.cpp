/*
 * main.cpp

 *
 *  Created on: 2020/01/15
 *      Author: yoneken
 *
 */
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Float64.h>

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;

ros::NodeHandle nh;

std_msgs::Float64 pressure;
ros::Publisher pub("air_pressure", &pressure);

#define ADCBUF_SIZE ((uint32_t) 1)
volatile uint16_t ADCBUF[ADCBUF_SIZE];
volatile static int adc_count = 0;
#define MEAN_NUM ((uint8_t) 4)
int adc_sum = 0;

short air_pressure_offset = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(pub);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBUF, ADCBUF_SIZE);
  HAL_TIM_Base_Start(&htim6);
}

void loop(void)
{
  nh.spinOnce();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // Every 50[ms] launched by Timer6
  if(adc_count < MEAN_NUM){
    adc_sum += ADCBUF[0];
    adc_count++;
  }else if(adc_count == MEAN_NUM){
	air_pressure_offset = adc_sum >> 2;
    adc_count++;
  }else{
    float voltage = (float)(ADCBUF[0] - air_pressure_offset) / (float)0x0fff * 3.3f;
    pressure.data = voltage / 2.7f * 1000.f / 10.f;
  }
  pub.publish(&pressure);
}
