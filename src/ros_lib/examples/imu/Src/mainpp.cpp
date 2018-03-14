/*
 * main.cpp

 *
 *  Created on: 2018/01/02
 *      Author: yoneken
 */
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#include "mpu9250.h"
#include <mainpp.h>
#include <queue>
#include <memory>
#include <ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Imu.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pub_imu("imu", &imu);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

class SPI_Mem{
public:
  unsigned short int chip;
  short data_num;
  std::unique_ptr<uint8_t[], std::default_delete<uint8_t[]> >data;
  void (*callback)(uint8_t dat[]);

  // send message
  SPI_Mem(unsigned short int chip_, uint8_t address_, short num_, const uint8_t data_[])
    : chip(chip_), data_num(num_), data(new uint8_t[1+num_]), callback(NULL)
  {
    uint8_t *dat = data.get();
    dat[0] = address_;
    memcpy(&(dat[1]), data_, num_);
  }

  // receive message
  SPI_Mem(unsigned short int chip_, uint8_t address_, void (*callback_)(uint8_t dat[]), short num_)
    : chip(chip_), data_num(num_), data(new uint8_t[1+num_]), callback(callback_)
  {
    uint8_t *dat = data.get();
    dat[0] = address_;
    memset(&(dat[1]), 0xff, num_);
  }

  virtual ~SPI_Mem(){}
};

volatile bool flag_transmit = false;
std::queue<std::unique_ptr<SPI_Mem>>spiq;
uint8_t rbuf[64];


void SPI_Mem_Transmit(void){
  HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&hspi1);
  if(!flag_transmit){
    flag_transmit = true;
    SPI_Mem *m = spiq.front().get();
    HAL_GPIO_WritePin(GPIOA, m->chip, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, m->data.get(), rbuf, 1+m->data_num);
  }
}

void SPI_Mem_Write(unsigned short int chip, uint8_t address, short num, const uint8_t data[]){
  spiq.push(std::unique_ptr<SPI_Mem>(new SPI_Mem(chip, address, num, data)));
  SPI_Mem_Transmit();
}

void SPI_Mem_Read(unsigned short int chip, uint8_t address, void (*callback)(uint8_t dat[]), short num){
  spiq.push(std::unique_ptr<SPI_Mem>(new SPI_Mem(chip, address | 0x80, callback, num)));
  SPI_Mem_Transmit();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  SPI_Mem *m = spiq.front().get();
  HAL_GPIO_WritePin(GPIOA, m->chip, GPIO_PIN_SET);
  if(m->callback != NULL) m->callback(&(rbuf[1]));
  uint8_t s = spiq.size();
  spiq.pop();
  flag_transmit = false;
  if(!spiq.empty()) SPI_Mem_Transmit();
}

void cb_whoami(uint8_t dat[]){
  uint8_t tmp = dat[0];	// expect MPU9250-0x71(113), MPU9255-0x73(115)
}

void cb_sense(uint8_t dat[]){
  short acc[3], temp, gyro[3];
  acc[0] = dat[0] << 8 | dat[1];
  acc[1] = dat[2] << 8 | dat[3];
  acc[2] = dat[4] << 8 | dat[5];
  temp = dat[6] << 8 | dat[7];
  gyro[0] = dat[8] << 8 | dat[9];
  gyro[1] = dat[10] << 8 | dat[11];
  gyro[2] = dat[12] << 8 | dat[13];
}

void cb_mag_sense(uint8_t dat[]){
  short magnet[3];
  magnet[0] = dat[0] | dat[1] << 8;
  magnet[1] = dat[2] | dat[3] << 8;
  magnet[2] = dat[4] | dat[5] << 8;
}

void setup(void)
{
  nh.initNode();
  //nh.advertise(thermo);

  // Initialize I2C Master mode of mpu9250
  USER_CTRL_REG uctr[1] = {0};
  uctr[0].I2C_MST_EN = 1;	// Enable the I2C Master I/F module
  uctr[0].I2C_IF_DIS = 1;	// Disable I2C Slave module and put the serial interface in SPI mode only
  //uctr[0].BYTE = 0x30;
  SPI_Mem_Write(CS_IMU_Pin, USER_CTRL, 1, (uint8_t*)uctr);	// expect 0x30(48)

  I2C_MST_CTRL_REG i2cmctr[1] = {0};
  i2cmctr[0].I2C_MST_CLK = _400kHz;	// Set I2c Master clock to 400kHz
  SPI_Mem_Write(CS_IMU_Pin, I2C_MST_CTRL, 1, (uint8_t*)i2cmctr);		// expect 0x0D(13)

  // Reset AK8963 (Magnetometer implemented on mpu9250) through I2C
  I2C_SLV0_ADDR_REG i2caddr[1] = {0};
  i2caddr[0].I2C_ID_0 = 0x0c;	// Since AK8963 is addressed on 0x0c
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_ADDR, 1, (uint8_t*)i2caddr);	// expect 0x0C(12)

  I2C_SLV0_REG_REG i2creg[1] = {0};
  i2creg[0].BYTE = CNTL2;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);	// expect 0x0B(11)

  AK8963_CNTL2_REG cntl2[1] = {0};
  cntl2[0].SRST = 1;	// Enable reset
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_DO, 1, (uint8_t*)cntl2);	// expect 0x01(1)

  I2C_SLV0_CTRL_REG i2cctr[1] = {0};
  i2cctr[0].I2C_SLV0_EN = 1;	// Eneble I2C
  i2cctr[0].I2C_SLV0_LENG = 1;	// Write 1 byte
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);	// expect 0x81(129)

  // Initialize AK8963 for continuous sensing mode through I2C
  i2creg[0].BYTE = CNTL;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);	// expect 0x0A(10)

  AK8963_CNTL1_REG cntl1[1] = {0};
  cntl1[0].MODE = CONT_MES_MODE2;	// continuous measuring mode
  cntl1[0].BIT = OUT_16BIT;			// measure 16 bit
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_DO, 1, (uint8_t*)cntl1);	// expect 0x16(22)

  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);	// expect 0x81(129)

  // porting from motion_driver
  inv_error_t result;
  result = mpu_init(&int_param);
  result = inv_init_mpl();

  inv_enable_quaternion();
  inv_enable_9x_sensor_fusion();

  inv_enable_fast_nomot();
  // inv_enable_motion_no_motion();
  // inv_set_no_motion_time(1000);

  inv_enable_gyro_tc();

  // inv_enable_in_use_auto_calibration();

  inv_enable_vector_compass_cal();
  inv_enable_magnetic_disturbance();

  result = inv_start_mpl();

  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  mpu_get_compass_fsr(&compass_fsr);

  inv_set_gyro_sample_rate(1000000L / gyro_rate);
  inv_set_accel_sample_rate(1000000L / gyro_rate);
  inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);

  inv_set_gyro_orientation_and_scale(
          inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)gyro_fsr<<15);
  inv_set_accel_orientation_and_scale(
          inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)accel_fsr<<15);
  inv_set_compass_orientation_and_scale(
          inv_orientation_matrix_to_scalar(compass_pdata.orientation),
          (long)compass_fsr<<15);

  dmp_load_motion_driver_firmware();
  dmp_set_orientation(
      inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  mpu_set_dmp_state(1);

}

void build_gyroacc(short *gyro, short *accel_short, unsigned long *timestamp, unsigned char *sensors)
{
  long accel[3], temperature;

  if(sensors & INV_XYZ_GYRO){
    inv_build_gyro(gyro, timestamp);
    if(new_temp) {
      new_temp = 0;
      mpu_get_temperature(&temperature, &timestamp);
      inv_build_temp(temperature, timestamp);
    }
  }

  if(sensors & INV_XYZ_ACCEL){
    accel[0] = (long)accel_short[0];
    accel[1] = (long)accel_short[1];
    accel[2] = (long)accel_short[2];
    inv_build_accel(accel, 0, timestamp);
  }

}

void loop(void)
{
  //SPI_Mem_Read(CS_IMU_Pin, WHO_AM_I, cb_whoami, 1);
  SPI_Mem_Read(CS_IMU_Pin, ACCEL_XOUT_H, cb_sense, 14);

  // Read magnetometer through I2C
  I2C_SLV0_ADDR_REG i2caddr[1] = {0};
  i2caddr[0].I2C_ID_0 = 0x0c;	// Since AK8963 is addressed on 0x0c
  i2caddr[0].I2C_SLV0_RNW = 1;	// read flag
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_ADDR, 1, (uint8_t*)i2caddr);	// expect 0x8C(140)

  I2C_SLV0_REG_REG i2creg[1] = {0};
  i2creg[0].BYTE = HXL;
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_REG, 1, (uint8_t*)i2creg);	// expect 0x03(3)

  I2C_SLV0_CTRL_REG i2cctr[1] = {0};
  i2cctr[0].I2C_SLV0_EN = 1;	// Eneble I2C
  i2cctr[0].I2C_SLV0_LENG = 7;	// Read 7 byte from HXL to ST2
  SPI_Mem_Write(CS_IMU_Pin, I2C_SLV0_CTRL, 1, (uint8_t*)i2cctr);	// expect 0x87(135)

  SPI_Mem_Read(CS_IMU_Pin, EXT_SENS_DATA, cb_mag_sense, 7);
  /*
  if(!flag){
    flag = true;

    tbuf[0] = WHO_AM_I | 0x80;
    tbuf[1] = 0xff;
    tbuf[2] = 0xff;
    tbuf[3] = 0xff;
    tbuf[4] = 0xff;
    tbuf[5] = 0xff;
    tbuf[6] = 0xff;
    tbuf[7] = 0xff;
    //tbuf[0] = ACCEL_XOUT_H | 0x80;

    HAL_GPIO_WritePin(GPIOA, CS_IMU_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tbuf, 1, 1000);
    HAL_SPI_Receive(&hspi1, rbuf, 8, 1000);	// expect MPU9250-0x71(113), MPU9255-0x73(115)
    //HAL_SPI_TransmitReceive(&hspi1, tbuf, rbuf, 8, 1000);
    HAL_GPIO_WritePin(GPIOA, CS_IMU_Pin, GPIO_PIN_SET);
  }
  */

  // porting
  short gyro[3], accel_short[3], sensors;
  unsigned char more;
  long accel[3], quat[4], temperature;

  dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
  build_gyroacc(gyro, accel_short, &sensor_timestamp, &sensors);

  if(sensors & INV_WXYZ_QUAT){
    inv_build_quat(quat, 0, sensor_timestamp);
  }

  if(more){
    mpu_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors, &more);
    build_gyroacc(gyro, accel_short, &sensor_timestamp, &sensors);
  }

  if(newdata){
    inv_execute_on_data();

    float float_data[3] = {0};
    long data[9];
    int8_t accuracy;
    unsigned long timestamp;
    if(inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)){
      // quaternion
    }
    if(inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp)){
      // acc
    }
    if(inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*)&timestamp)){
      // gyro
    }
    if(inv_get_sensor_type_compass(data, &accuracy, (inv_time_t*)&timestamp)){
      // compass
    }
    if(inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)){
      // linear acc
    }
  }

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

  nh.spinOnce();
  HAL_Delay(1000);
}

