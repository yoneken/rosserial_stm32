/* * mpu9250.h
 *
 *  Created on: 2018/02/01
 *      Author: yoneken
 */
#ifndef _MPU9250_H_
#define _MPU9250_H_

enum MPU9250_Reg{
  SELF_TEST_X_GYRO = 0x00,  // R/W
  SELF_TEST_Y_GYRO = 0x01,  // R/W
  SELF_TEST_Z_GYRO = 0x02,  // R/W
  SELF_TEST_X_ACCEL = 0x0D,  // R/W
  SELF_TEST_Y_ACCEL = 0x0E,  // R/W
  SELF_TEST_Z_ACCEL = 0x0F,  // R/W
  XG_OFFSET_H = 0x13,  // R/W
  XG_OFFSET_L = 0x14,  // R/W
  YG_OFFSET_H = 0x15,  // R/W
  YG_OFFSET_L = 0x16,  // R/W
  ZG_OFFSET_H = 0x17,  // R/W
  ZG_OFFSET_L = 0x18,  // R/W
  SMPLRT_DIV = 0x19,  // R/W
  CONFIG = 0x1A,   // R/W
  GYRO_CONFIG = 0x1B, // R/W
  ACCEL_CONFIG = 0x1C, // R/W
  ACCEL_CONFIG2 = 0x1D, // R/W
  LP_ACCEL_ODR = 0x1E, // R/W
  WOM_THR = 0x1F, // R/W
  FIFO_EN = 0x23,// R/W
  I2C_MST_CTRL = 0x24, // R/W
  I2C_SLV0_ADDR = 0x25, // R/W
  I2C_SLV0_REG = 0x26,  // R/W
  I2C_SLV0_CTRL = 0x27,  // R/W
  I2C_SLV1_ADDR = 0x28, // R/W
  I2C_SLV1_REG = 0x29,  // R/W
  I2C_SLV1_CTRL = 0x2A,  // R/W
  I2C_SLV2_ADDR = 0x2B, // R/W
  I2C_SLV2_REG = 0x2C,  // R/W
  I2C_SLV2_CTRL = 0x2D,  // R/W
  I2C_SLV3_ADDR = 0x2E, // R/W
  I2C_SLV3_REG = 0x2F,  // R/W
  I2C_SLV3_CTRL = 0x30,  // R/W
  I2C_SLV4_ADDR = 0x31, // R/W
  I2C_SLV4_REG = 0x32,  // R/W
  I2C_SLV4_DO = 0x33,  // R/W
  I2C_SLV4_CTRL = 0x34,  // R/W
  I2C_SLV4_DI = 0x35,  // R
  I2C_MST_STATUS = 0x36,  // R
  INT_PIN_CFG = 0x37,  // R/W
  INT_ENABLE = 0x38,  // R/W
  INT_STATUS = 0x3A,  // R
  ACCEL_XOUT_H = 0x3B,  // R
  ACCEL_XOUT_L = 0x3C,  // R
  ACCEL_YOUT_H = 0x3D,  // R
  ACCEL_YOUT_L = 0x3E,  // R
  ACCEL_ZOUT_H = 0x3F,  // R
  ACCEL_ZOUT_L = 0x40,  // R
  TEMP_OUT_H = 0x41,  // R
  TEMP_OUT_L = 0x42,  // R
  GYRO_XOUT_H = 0x43,  // R
  GYRO_XOUT_L = 0x44,  // R
  GYRO_YOUT_H = 0x45,  // R
  GYRO_YOUT_L = 0x46,  // R
  GYRO_ZOUT_H = 0x47,  // R
  GYRO_ZOUT_L = 0x48,  // R
  EXT_SENS_DATA = 0x49,  // R
  I2C_SLV0_DO = 0x63,  // R/W
  I2C_SLV1_DO = 0x64,  // R/W
  I2C_SLV2_DO = 0x65,  // R/W
  I2C_SLV3_DO = 0x66,  // R/W
  I2C_MST_DELAY_CTRL = 0x67,  // R/W
  SIGNAL_PATH_RESET = 0x68,  // R/W
  MOT_DETECT_CTRL = 0x69,  // R/W
  USER_CTRL = 0x6A,  // R/W
  PWR_MGMT_1 = 0x6B,// R/W
  PWR_MGMT_2 = 0x6C,// R/W
  FIFO_COUNTH = 0x72,  // R/W
  FIFO_COUNTL = 0x73,  // R/W
  FIFO_R_W = 0x74,  // R/W
  WHO_AM_I = 0x75,  // R
  XA_OFFSET_H = 0x77,  // R/W
  XA_OFFSET_L = 0x78,  // R/W
  YA_OFFSET_H = 0x7A,  // R/W
  YA_OFFSET_L = 0x7B,  // R/W
  ZA_OFFSET_H = 0x7D,  // R/W
  ZA_OFFSET_L = 0x7E,  // R/W
};

union SELF_TEST_X_GYRO_REG{
  uint8_t BYTE:8;
  uint8_t xg_st_data:8;
};

union SELF_TEST_Y_GYRO_REG{
  uint8_t BYTE:8;
  uint8_t yg_st_data:8;
};

union SELF_TEST_Z_GYRO_REG{
  uint8_t BYTE:8;
  uint8_t zg_st_data:8;
};

union SELF_TEST_X_ACCEL_REG{
  uint8_t BYTE:8;
  uint8_t XA_ST_DATA:8;
};

union SELF_TEST_Y_ACCEL_REG{
  uint8_t BYTE:8;
  uint8_t YA_ST_DATA:8;
};

union SELF_TEST_Z_ACCEL_REG{
  uint8_t BYTE:8;
  uint8_t ZA_ST_DATA:8;
};

union XG_OFFSET_H_REG{
  uint8_t BYTE:8;
  uint8_t X_OFFS_USR:8;
};

union XG_OFFSET_L_REG{
  uint8_t BYTE:8;
  uint8_t X_OFFS_USR:8;
};

union YG_OFFSET_H_REG{
  uint8_t BYTE:8;
  uint8_t Y_OFFS_USR:8;
};

union YG_OFFSET_L_REG{
  uint8_t BYTE:8;
  uint8_t Y_OFFS_USR:8;
};

union ZG_OFFSET_H_REG{
  uint8_t BYTE:8;
  uint8_t Z_OFFS_USR:8;
};

union ZG_OFFSET_L_REG{
  uint8_t BYTE:8;
  uint8_t Z_OFFS_USR:8;
};

union SMPLRT_DIV_REG{
  uint8_t BYTE:8;
  uint8_t SMPLRT_DIV:8;
};

union CONFIG_REG{
  uint8_t BYTE:8;
  struct{
     uint8_t DLPF_CFG:3;
     uint8_t EXT_SYNC_SET:3;
     uint8_t FIFO_MODE:1;
     uint8_t reserved:1;
  };
};

union GYRO_CONFIG_REG{
  uint8_t BYTE:8;
  struct{
     uint8_t Fchoice_b:2;
     uint8_t reserved:1;
     uint8_t GYRO_FS_SEL:2;
     uint8_t ZGYRO_Cten:1;
     uint8_t YGYRO_Cten:1;
     uint8_t XGYRO_Cten:1;
  };
};

union ACCEL_CONFIG_REG{
  uint8_t BYTE:8;
  struct{
     uint8_t reserved:3;
     uint8_t ACCEL_FS_SEL:2;
     uint8_t az_st_en:1;
     uint8_t ay_st_en:1;
     uint8_t ax_st_en:1;
  };
};

union ACCEL_CONFIG2_REG{
  uint8_t BYTE:8;
  struct{
     uint8_t A_DLPFCFG:3;
     uint8_t accel_fchoice_b:1;
     uint8_t reserved:4;
  };
};

union LP_ACCEL_ODR_REG{
  uint8_t BYTE:8;
  struct{
     uint8_t lposc_clksel:4;
     uint8_t reserved:4;
  };
};

union WOM_THR_REG{
  uint8_t BYTE:8;
  uint8_t WOM_Threshold:8;
};

union FIFO_EN_REG{
  uint8_t BYTE:8;
  struct{
     uint8_t SLV0:1;
     uint8_t SLV1:1;
     uint8_t SLV2:1;
     uint8_t ACCEL:1;
     uint8_t GYRO_ZOUT:1;
     uint8_t GYRO_YOUT:1;
     uint8_t GYRO_XOUT:1;
     uint8_t TEMP_FIFO_EN:1;
  };
};

union I2C_MST_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_MST_CLK:4;
    uint8_t I2C_MST_P_NSR:1;
    uint8_t SLV_3_FIFO_EN:1;
    uint8_t WAIT_FOR_ES:1;
    uint8_t MULT_MST_EN:1;
  };
};

enum I2C_MST_CLK_SPEED{
  _348kHz = 0,
  _333kHz = 1,
  _320kHz = 2,
  _308kHz = 3,
  _296kHz = 4,
  _286kHz = 5,
  _276kHz = 6,
  _267kHz = 7,
  _258kHz = 8,
  _500kHz = 9,
  _471kHz = 10,
  _444kHz = 11,
  _421kHz = 12,
  _400kHz = 13,
  _381kHz = 14,
  _364kHz = 15,
};

union I2C_SLV0_ADDR_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_ID_0:7;
    uint8_t I2C_SLV0_RNW:1;
  };
};

union I2C_SLV0_REG_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV0_REG:8;
};

union I2C_SLV0_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_SLV0_LENG:4;
    uint8_t I2C_SLV0_GRP:1;
    uint8_t I2C_SLV0_REG_DIS:1;
    uint8_t I2C_SLV0_BYTE_SW:1;
    uint8_t I2C_SLV0_EN:1;
  };
};

union I2C_SLV1_ADDR_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_ID_1:7;
    uint8_t I2C_SLV1_RNW:1;
  };
};

union I2C_SLV1_REG_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV1_REG:8;
};

union I2C_SLV1_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_SLV1_LENG:4;
    uint8_t I2C_SLV1_GRP:1;
    uint8_t I2C_SLV1_REG_DIS:1;
    uint8_t I2C_SLV1_BYTE_SW:1;
    uint8_t I2C_SLV1_EN:1;
  };
};

union I2C_SLV2_ADDR_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_ID_2:7;
    uint8_t I2C_SLV2_RNW:1;
  };
};

union I2C_SLV2_REG_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV2_REG:8;
};

union I2C_SLV2_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_SLV2_LENG:4;
    uint8_t I2C_SLV2_GRP:1;
    uint8_t I2C_SLV2_REG_DIS:1;
    uint8_t I2C_SLV2_BYTE_SW:1;
    uint8_t I2C_SLV2_EN:1;
  };
};

union I2C_SLV3_ADDR_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_ID_3:7;
    uint8_t I2C_SLV3_RNW:1;
  };
};

union I2C_SLV3_REG_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV3_REG:8;
};

union I2C_SLV3_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_SLV3_LENG:4;
    uint8_t I2C_SLV3_GRP:1;
    uint8_t I2C_SLV3_REG_DIS:1;
    uint8_t I2C_SLV3_BYTE_SW:1;
    uint8_t I2C_SLV3_EN:1;
  };
};

union I2C_SLV4_ADDR_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_ID_4:7;
    uint8_t I2C_SLV4_RNW:1;
  };
};

union I2C_SLV4_REG_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV4_REG:8;
};

union I2C_SLV4_DO_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV4_DO:8;
};

union I2C_SLV4_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_MST_DLY:5;
    uint8_t I2C_SLV4_REG_DIS:1;
    uint8_t I2C_SLV4_DONE_INT_EN:1;
    uint8_t I2C_SLV4_EN:1;
  };
};

union I2C_SLV4_DI_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV4_DI:8;
};

union I2C_MST_STATUS_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_SLV0_NACK:1;
    uint8_t I2C_SLV1_NACK:1;
    uint8_t I2C_SLV2_NACK:1;
    uint8_t I2C_SLV3_NACK:1;
    uint8_t I2C_SLV4_NACK:1;
    uint8_t I2C_LOST_ARB:1;
    uint8_t I2C_SLV4_DONE:1;
    uint8_t PASS_THROUGH:1;
  };
};

union INT_PIN_CFG_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t reserved:1;
    uint8_t BYPASS_EN:1;
    uint8_t FSYNC_INT_MODE_EN:1;
    uint8_t ACTL_FSYNC:1;
    uint8_t INT_ANYRD_2CLEAR:1;
    uint8_t LATCH_INT_EN:1;
    uint8_t OPEN:1;
    uint8_t ACTL:1;
  };
};

union INT_ENABLE_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t RAW_RDY_EN:1;
    uint8_t reserved:2;
    uint8_t FSYNC_INT_EN:1;
    uint8_t FIFO_OFLOW_EN:1;
    uint8_t reserved2:1;
    uint8_t WOM_EN:1;
    uint8_t reserved3:1;
  };
};

union INT_STATUS_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t RAW_DATA_RDY_INT:1;
    uint8_t reserved3:2;
    uint8_t FSYNC_INT:1;
    uint8_t FIFO_OFLOW_INT:1;
    uint8_t reserve2:1;
    uint8_t WOM_INT:1;
    uint8_t reserved:1;
  };
};

union ACCEL_XOUT_H_REG{
  uint8_t BYTE:8;
  uint8_t ACCEL_XOUT_H:8;
};

union ACCEL_XOUT_L_REG{
  uint8_t BYTE:8;
  uint8_t ACCEL_XOUT_L:8;
};

union ACCEL_YOUT_H_REG{
  uint8_t BYTE:8;
  uint8_t ACCEL_YOUT_H:8;
};

union ACCEL_YOUT_L_REG{
  uint8_t BYTE:8;
  uint8_t ACCEL_YOUT_L:8;
};

union ACCEL_ZOUT_H_REG{
  uint8_t BYTE:8;
  uint8_t ACCEL_ZOUT_H:8;
};

union ACCEL_ZOUT_L_REG{
  uint8_t BYTE:8;
  uint8_t ACCEL_ZOUT_L:8;
};

union TEMP_OUT_H_REG{
  uint8_t BYTE:8;
  uint8_t TEMP_OUT_H:8;
};

union TEMP_OUT_L_REG{
  uint8_t BYTE:8;
  uint8_t TEMP_OUT_L:8;
};

union GYRO_XOUT_H_REG{
  uint8_t BYTE:8;
  uint8_t GYRO_XOUT_H:8;
};

union GYRO_XOUT_L_REG{
  uint8_t BYTE:8;
  uint8_t GYRO_XOUT_L:8;
};

union GYRO_YOUT_H_REG{
  uint8_t BYTE:8;
  uint8_t GYRO_YOUT_H:8;
};

union GYRO_YOUT_L_REG{
  uint8_t BYTE:8;
  uint8_t GYRO_YOUT_L:8;
};

union GYRO_ZOUT_H_REG{
  uint8_t BYTE:8;
  uint8_t GYRO_ZOUT_H:8;
};

union GYRO_ZOUT_L_REG{
  uint8_t BYTE:8;
  uint8_t GYRO_ZOUT_L:8;
};

union I2C_SLV0_DO_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV0_DO:8;
};

union I2C_SLV1_DO_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV1_DO:8;
};

union I2C_SLV2_DO_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV2_DO:8;
};

union I2C_SLV3_DO_REG{
  uint8_t BYTE:8;
  uint8_t I2C_SLV3_DO:8;
};

union I2C_MST_DELAY_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t I2C_SLV0_DLY_EN:1;
    uint8_t I2C_SLV1_DLY_EN:1;
    uint8_t I2C_SLV2_DLY_EN:1;
    uint8_t I2C_SLV3_DLY_EN:1;
    uint8_t I2C_SLV4_DLY_EN:1;
    uint8_t reserved:2;
    uint8_t DELAY_ES_SHADOW:1;
  };
};

union SIGNAL_PATH_RESET_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t TEMP_RST:1;
    uint8_t ACCEL_RST:1;
    uint8_t GYRO_RST:1;
    uint8_t reserved:5;
  };
};

union MOT_DETECT_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t reserved:6;
    uint8_t ACCEL_INTEL_MODE:1;
    uint8_t ACCEL_INTEL_EN:1;
  };
};

union USER_CTRL_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t SIG_COND_RST:1;
    uint8_t I2C_MST_RST:1;
    uint8_t FIFO_RST:1;
    uint8_t reserved2:1;
    uint8_t I2C_IF_DIS:1;
    uint8_t I2C_MST_EN:1;
    uint8_t FIFO_EN:1;
    uint8_t reserved:1;
  };
};

union PWR_MGMT_1_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t CLKSEL:3;
    uint8_t PD_PTAT:1;
    uint8_t GYRO_STANDBY:1;
    uint8_t CYCLE:1;
    uint8_t SLEEP:1;
    uint8_t H_RESET:1;
  };
};

union PWR_MGMT_2_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t DIS_ZG:1;
    uint8_t DIS_YG:1;
    uint8_t DIS_XG:1;
    uint8_t DIS_ZA:1;
    uint8_t DIS_YA:1;
    uint8_t DIS_XA:1;
    uint8_t reserved:2;
  };
};

union FIFO_COUNTH_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t FIFO_CNT:5;
    uint8_t reserved:3;
  };
};

union FIFO_COUNTL_REG{
  uint8_t BYTE:8;
  uint8_t FIFO_CNT:8;
};

union FIFO_R_W_REG{
  uint8_t BYTE:8;
  uint8_t D:8;
};

union WHO_AM_I_REG{
  uint8_t BYTE:8;
  uint8_t WHOAMI:8;
};

union XA_OFFSET_H_REG{
  uint8_t BYTE:8;
  uint8_t XA_OFFS:8;
};

union XA_OFFSET_L_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t reserved:1;
    uint8_t XA_OFFS:7;
  };
};

union YA_OFFSET_H_REG{
  uint8_t BYTE:8;
  uint8_t YA_OFFS:8;
};

union YA_OFFSET_L_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t reserved:1;
    uint8_t YA_OFFS:7;
  };
};

union ZA_OFFSET_H_REG{
  uint8_t BYTE:8;
  uint8_t ZA_OFFS:8;
};

union ZA_OFFSET_L_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t reserved:1;
    uint8_t ZA_OFFS:7;
  };
};

enum AK8963_Reg{
  WIA = 0x00,  // R
  INFO = 0x01,  // R
  ST1 = 0x02,  // R
  HXL = 0x03,  // R
  HXH = 0x04,  // R
  HYL = 0x05,  // R
  HYH = 0x06,  // R
  HZL = 0x07,  // R
  HZH = 0x08,  // R
  ST2 = 0x09,  // R
  CNTL = 0x0a,  // R/W
  CNTL2 = 0x0b,  // R/W
  ASTC = 0x0c,  // R/W
  TS1 = 0x0d,  // R/W
  TS2 = 0x0e,  // R/W
  I2CDIS = 0x0f,  // R/W
  ASAX = 0x10,  // R
  ASAY = 0x11,  // R
  ASAZ = 0x12,  // R
};

union AK8963_WIA_REG{
  uint8_t BYTE:8;
  uint8_t WIA:8;
};

union AK8963_INFO_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t INFO0:1;
    uint8_t INFO1:1;
    uint8_t INFO2:1;
    uint8_t INFO3:1;
    uint8_t INFO4:1;
    uint8_t INFO5:1;
    uint8_t INFO6:1;
    uint8_t INFO7:1;
  };
};

union AK8963_ST1_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t DRDY:1;
    uint8_t DOR:1;
    uint8_t reserved:6;
  };
};

union AK8963_ST2_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t reserved:3;
    uint8_t HOFL:1;
    uint8_t BITM:1;
    uint8_t reserved2:3;
  };
};

union AK8963_CNTL1_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t MODE:4;
    uint8_t BIT:1;
    uint8_t reserved:3;
  };
};

enum AK8963_CNTL1_MODE{
  POW_DWN_MODE = 0x00,
  SINGLE_MES_MODE = 0x01,
  CONT_MES_MODE1 = 0x02,
  CONT_MES_MODE2 = 0x06,
  EXT_TRIG_MES_MODE = 0x04,
  SELF_TEST_MODE = 0x08,
  FUSE_ROM_ACC_MODE = 0x0f,
};

enum AK8963_CNTL1_BIT{
  OUT_14BIT = 0x0,
  OUT_16BIT = 0x1,
};

union AK8963_CNTL2_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t SRST:1;
    uint8_t reserved:7;
  };
};

union AK8963_ASTC_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t reserved:6;
    uint8_t SELF:1;
    uint8_t reserved2:1;
  };
};

enum AK8963_I2CDIS{
  AK8963_I2CDIS = 0x1b,
};


#ifdef __cplusplus
extern "C" {
#endif

// Define functions

#ifdef __cplusplus
};
#endif


#endif /* _MPU9250_H_ */
