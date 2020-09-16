/*
 * @file mainpp.cpp
 *
 *  Created on: Apr 12, 2020
 *  @author fofolevrai
 *
 *      @brief This example extract pressure and temperature data given by the LPS22HB sensor
 *      using the DMA I2C, DMA UART and Timer
 */

#include "main.h"
#include <std_msgs/String.h>
#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_hal_i2c.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <ros.h>
#include <ros/time.h>
#include "lps22hb.h"
#include "mainpp.h"

uint8_t tu8_ctrl_reg1_data[2] = {LPS22HB_I2C_REGISTER_ADDR_CTRL_REG1, 0x1EU};	/* LPS22HB data rate set to 1 Hz */
uint8_t tu8_ctrl_reg2_data[2] = {LPS22HB_I2C_REGISTER_ADDR_CTRL_REG2, 0x50U};	/*	FIFO On and multiple reading allowed */
uint8_t tu8_fifo_ctrl_data1[2] = {LPS22HB_I2C_REGISTER_ADDR_FIFO_CTRL, 0x00U};	/*	LPS22HB FIFO reset and flushed */
uint8_t tu8_fifo_ctrl_data2[2] = {LPS22HB_I2C_REGISTER_ADDR_FIFO_CTRL, 0x40U};	/*	FIFO set to stream mode */

static bool m_b_i2c_lps22hb_initializationSet = false;							/*	Does the LPS22HB initialization is been set */
static bool m_b_i2c_RxComplete = false;											/*	Did we receive a new i2c frame to compute	*/
static Lps322hb_out_data_t m_st_lps22hbData = (const Lps322hb_out_data_t) {0};	/*	Will contain LPS22HB raw pressure and temperature data	*/
sensor_msgs::Temperature m_ros_LPS22HB_Temperature;								/*	ROS temperature frame	*/
sensor_msgs::FluidPressure m_ros_LPS22HB_Pressure;								/*	ROS pressure frame	*/

ros::NodeHandle nh;

ros::Publisher m_ros_LPS22HB_Pressure_Publisher("LPS22HB_Pressure", &m_ros_LPS22HB_Pressure);				/*	Declare ROS pressure publisher	*/
ros::Publisher m_ros_LPS22HB_Temperature_Publisher("LPS22HB_Temperature", &m_ros_LPS22HB_Temperature);		/*	Declare ROS temperature publisher	*/


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2chandle)
{
	Error_Handler();
}
/**
 * @brief Call once I2C transmission is complete
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//	Check we are not in initialization phase and i2c bus is free
	if(m_b_i2c_lps22hb_initializationSet && (HAL_I2C_STATE_READY == HAL_I2C_GetState(hi2c)))
	{
		//	DMA pressure and temperature I2C read request
		if(HAL_OK != HAL_I2C_Master_Receive_DMA(hi2c, hi2c->Devaddress, m_st_lps22hbData.rawData, sizeof(Lps322hb_out_data_t)))
		{
			//	Fall here if error on I2C bus
			Error_Handler();
		}
	}
}

/**
 * @brief Call once I2C reception is done
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//	Check we are not in initialization phase and i2c bus is free
	if(m_b_i2c_lps22hb_initializationSet && (HAL_I2C_STATE_READY == HAL_I2C_GetState(hi2c)))
	{
		//	Inform (main) we have new pressure and temperature data to compute
		m_b_i2c_RxComplete = true;
	}
}

/**
 * @brief Timer function called every 1 second (Defined with STM32CubeMX)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t l_tu8_press_out_data[1] = {LPS22HB_I2C_REGISTER_ADDR_PRESS_OUT_XL};

	//	Check we are not in initialization phase
	if(m_b_i2c_lps22hb_initializationSet)
	{
		//	Send the register from which we want to read of
		if(HAL_OK != HAL_I2C_Master_Transmit_DMA(&hi2c1, LPS22HB_I2C_DEVICE_ADDRESS, l_tu8_press_out_data, 1))
		{
			//	Fall here if we had an error on I2C bus
			HAL_TIM_Base_Stop_IT(htim);
			Error_Handler();
		}
	}

	//	Toggle the green LD on the Nucleo board
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void setup(void)
{
	/**********************************
	 *	ROS initialization section
	 **********************************/
	 nh.initNode();
	 m_ros_LPS22HB_Pressure.header.frame_id = "/LPS22HB";
	 m_ros_LPS22HB_Temperature.header.frame_id = "/LPS22HB";
	 nh.advertise(m_ros_LPS22HB_Pressure_Publisher);
	 nh.advertise(m_ros_LPS22HB_Temperature_Publisher);

	 /*********************************
	  * LPS22HB configuration section
	  *********************************/
	while(HAL_OK != HAL_I2C_Master_Transmit_DMA(&hi2c1, LPS22HB_I2C_DEVICE_ADDRESS, tu8_ctrl_reg1_data, 2))
	{
		if(HAL_I2C_GetError(&hi2c1))
		{
			/*
			 * Error_Handler() function is called when Timout error occurs.
			 * When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
			 * Master restarts communication
			 */
			Error_Handler();
		}
	}

	 /*##-3- Wait for the end of the transfer ###################################*/
	  /*  Before starting a new communication transfer, you need to check the current
	      state of the peripheral; if it’s busy you need to wait for the end of current
	      transfer before starting a new one.
	      For simplicity reasons, this example is just waiting till the end of the
	      transfer, but application may perform other tasks while transfer operation
	      is ongoing.
	      */
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	while(HAL_OK != HAL_I2C_Master_Transmit_DMA(&hi2c1, LPS22HB_I2C_DEVICE_ADDRESS, tu8_ctrl_reg2_data, 2))
	{
		if(HAL_I2C_GetError(&hi2c1))
		{
			/*
			 * Error_Handler() function is called when Timout error occurs.
			 * When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
			 * Master restarts communication
			 */
			Error_Handler();
		}
	}

	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	while(HAL_OK != HAL_I2C_Master_Transmit_DMA(&hi2c1, LPS22HB_I2C_DEVICE_ADDRESS, tu8_fifo_ctrl_data1, 2))
	{
		if(HAL_I2C_GetError(&hi2c1))
		{
			/*
			 * Error_Handler() function is called when Timout error occurs.
			 * When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
			 * Master restarts communication
			 */
			Error_Handler();
		}
	}

	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	while(HAL_OK != HAL_I2C_Master_Transmit_DMA(&hi2c1, LPS22HB_I2C_DEVICE_ADDRESS, tu8_fifo_ctrl_data2, 2))
	{
		if(HAL_I2C_GetError(&hi2c1))
		{
			/*
			 * Error_Handler() function is called when Timout error occurs.
			 * When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
			 * Master restarts communication
			 */
			Error_Handler();
		}
	}

	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	//	Initialization finished
	m_b_i2c_lps22hb_initializationSet = true;

	//	Start time 1 Hz TIM3 timer
	HAL_TIM_Base_Start_IT(&htim3);
}

void loop(void)
{
	//	Check we've received new data from I2C
	if(m_b_i2c_RxComplete)
	{
		//	Set time stamp
		m_ros_LPS22HB_Pressure.header.stamp = nh.now();
		m_ros_LPS22HB_Temperature.header.stamp = nh.now();

		//	Convert raw pressure and temperature to human understanding
		///////////////////////////////////////////////////////////////
		//	WARNING : THIS CONVERTION DO NOT HANDLE NEGATIVE VALUES	 //
		///////////////////////////////////////////////////////////////
		m_ros_LPS22HB_Pressure.fluid_pressure = (double) ((double)((m_st_lps22hbData.pressureData[2] << 16) | (m_st_lps22hbData.pressureData[1] << 8)| (m_st_lps22hbData.pressureData[0]))/4096);
		m_ros_LPS22HB_Temperature.temperature = (double) ((double)((m_st_lps22hbData.temperatureData[1] << 8)| (m_st_lps22hbData.temperatureData[0]))/100);

		//	Publish ROS pressure frame over UART
		m_ros_LPS22HB_Pressure_Publisher.publish(&m_ros_LPS22HB_Pressure);
		//	Publish ROS temperarture frame over UART
		m_ros_LPS22HB_Temperature_Publisher.publish(&m_ros_LPS22HB_Temperature);
		//	Toggle an external LED (connected to A0 on Nucleo)
		HAL_GPIO_TogglePin(ExtLED_GPIO_Port, ExtLED_Pin);

		//	Compute complete, we can fetch a new values from LPS22HB
		m_b_i2c_RxComplete = false;
	}

	nh.spinOnce();

	HAL_Delay(1000);
}



