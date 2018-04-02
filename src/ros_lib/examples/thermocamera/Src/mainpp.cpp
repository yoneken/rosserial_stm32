/*
 * main.cpp

 *
 *  Created on: 2018/01/02
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "GridEye.h"
#include "sensor_msgs/Image.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

ros::NodeHandle nh;

uint8_t rbuf[8 * 8 * 2];
uint16_t temp_image[8 * 8];
volatile bool flag = false;
sensor_msgs::Image img;
ros::Publisher thermo("thermo", &img);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
  thermo.publish(&img);
  flag = false;
}

void setup(void)
{
  nh.initNode();
  nh.advertise(thermo);

  img.height = 8;
  img.width = 8;
  img.encoding = "mono16";
  img.step = 8 * sizeof(uint16_t);
  img.data_length = 8 * 8 * sizeof(uint16_t);
  img.data = rbuf;
}

void loop(void)
{
  if(!flag){
    flag = true;
    HAL_I2C_Mem_Read_DMA(&hi2c1, (uint16_t)(0x69<<1), TDAT, I2C_MEMADD_SIZE_8BIT, rbuf, 128);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
  }

  nh.spinOnce();
  HAL_Delay(100);
}

