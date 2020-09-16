/**
  ******************************************************************************
  * @file    lps22hb.h
  * @author  fofolevrai
  * @brief   LPS22HB pressure and temperature sensor helper file.
  *          This file provides I2C communication parameters and interfaces:
  *           + I2C address of the device
  *           + Register addresses
  *           + Some helper functions
  *
  *@History	Apr 13, 2020 : Document creation
  *@History Apr 17, 2020 : First implementation with LPS22HB data extraction
  */
#ifndef LPS22HB_DRIVER_LPS22HB_H_
#define LPS22HB_DRIVER_LPS22HB_H_

/**
 * @brief Set the LPS22HB IC2 address regarding SDO connection
 */
#define LPS22HB_DRIVER_LPS22HB_SD0_CONNECTED_TO_VDD			0x01U

#if		LPS22HB_DRIVER_LPS22HB_SD0_CONNECTED_TO_VDD
#define LPS22HB_I2C_DEVICE_ADDRESS				0xBAU	/* LPS22HB I2C device address = $BA when SDO connected to Vdd */
#else
#define LPS22HB_I2C_DEVICE_ADDRESS				0xB8U	/* LPS22HB I2C device address = $B8 when SDO connected to GND */
#endif	/*	LPS22HB_VDD_CONNECTED_TO_SDO */


/**
 * Register mapping section
 */
#define LPS22HB_I2C_REGISTER_ADDR_INTERRUPT_CFG	0x0BU	/*	Interrupt register address */
#define LPS22HB_I2C_REGISTER_ADDR_THS_P_L		0x0CU	/*	Low significant pressure threshold register address */
#define LPS22HB_I2C_REGISTER_ADDR_THS_P_H		0x0DU	/*	High significant pressure threshold register address */
#define LPS22HB_I2C_REGISTER_ADDR_WHO_AM_I		0x0FU	/*	Who am I register register address */
#define LPS22HB_I2C_REGISTER_ADDR_CTRL_REG1		0x10U	/*	Control register 1 register address */
#define LPS22HB_I2C_REGISTER_ADDR_CTRL_REG2		0x11U	/*	Control register 2 register address */
#define LPS22HB_I2C_REGISTER_ADDR_CTRL_REG3		0x12U	/*	Control register 3 register address */
#define LPS22HB_I2C_REGISTER_ADDR_FIFO_CTRL		0x14U	/*	FIFO configuration register	address */
#define LPS22HB_I2C_REGISTER_ADDR_REF_P_XL		0x15U	/*	Least significant reference pressure register address */
#define LPS22HB_I2C_REGISTER_ADDR_REF_P_L		0x16U	/*	Mid significant reference pressure register address	*/
#define LPS22HB_I2C_REGISTER_ADDR_REF_P_H		0x17U	/*	High significant reference pressure register address */
#define LPS22HB_I2C_REGISTER_ADDR_RPDS_L		0x18U	/*	Low pressure offset register address */
#define LPS22HB_I2C_REGISTER_ADDR_RPDS_H		0x19U	/*	High pressure offset register address */
#define LPS22HB_I2C_REGISTER_ADDR_RES_CONF		0x1AU	/*	Resolution register address */
#define LPS22HB_I2C_REGISTER_ADDR_INT_SOURCE	0x25U	/*	Interrupt register address	*/
#define LPS22HB_I2C_REGISTER_ADDR_FIFO_STATUS	0x26U	/*	FIFO status register address */
#define LPS22HB_I2C_REGISTER_ADDR_STATUS		0x27U	/*	Status register address */
#define LPS22HB_I2C_REGISTER_ADDR_PRESS_OUT_XL	0x28U	/*	Least significant pressure output data register */
#define LPS22HB_I2C_REGISTER_ADDR_PRESS_OUT_L	0x29U	/*	Mid significant pressure output data register */
#define LPS22HB_I2C_REGISTER_ADDR_PRESS_OUT_H	0x2AU	/*	High significant pressure output register data */
#define LPS22HB_I2C_REGISTER_ADDR_TEMP_OUT_L	0x2BU 	/*	Low significant temperature output data register */
#define LPS22HB_I2C_REGISTER_ADDR_TEMP_OUT_H	0x2CU	/*	High significant temperature output data register */
#define LPS22HB_I2C_REGISTER_ADDR_LPFP_RES		0x33U	/*	Filter reset register address */



#endif /* LPS22HB_DRIVER_LPS22HB_H_ */
