/* * GridEye.h
 *
 *  Created on: 2017/11/27
 *      Author: yoneken
 */
#ifndef _GRIDEYE_H_
#define _GRIDEYE_H_

#include "stm32f3xx_hal.h"

enum GridEye_Reg{
  PCLT = 0x00,  // R/W
  RST = 0x01,   // W
  FPSC = 0x02,  // R/W
  INTC = 0x03,  // R/W
  STAT = 0x04,  // R
  SCLR = 0x05,  // W
  AVE = 0x07,   // R/W
  INTHL = 0x08, // R/W
  INTHH = 0x09, // R/W
  INTLL = 0x0a, // R/W
  INTLH = 0x0b,// R/W
  INTSL = 0x0c, // R/W
  INTSH = 0x0d, // R/W
  TTHL = 0x0e,  // R
  TTHH = 0x0f,  // R
  INT0 = 0x10,  // R
  INT1 = 0x11,  // R
  INT2 = 0x12,  // R
  INT3 = 0x13,  // R
  INT4 = 0x14,  // R
  INT5 = 0x15,  // R
  INT6 = 0x16,  // R
  INT7 = 0x17,  // R
  SETAVE = 0x1f,// W
  TDAT = 0x80,  // R
};

union PCLT_REG{
  uint8_t BYTE:8;
};
enum PCLT_MODE{
  NORMAL = 0x00,
  SLEEP = 0x10,
  STAND_BY60 = 0x20,
  STAND_BY10 = 0x21,
};

union RST_REG{
  uint8_t BYTE:8;
};
enum RST_CMD{
  FLAG_RST = 0x30,
  INIT_RST = 0x3f,
};

union FPSC_REG{
  uint8_t BYTE:8;
  struct{
     uint8_t dummy:7;
     uint8_t BIT_FPS:1;
  };
};
enum FPS_MODE{
  FPS_1 = 1,
  FPS_10 = 0,
};

union INTC_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t dummy:6;
    uint8_t BIT_IMTMOD:1;
    uint8_t BIT_INTEN:1;
  };
};
enum INT_MOD{
  INT_DISABLE = 0,
  INT_ABS = 3,
  INT_DIFF = 1,
};

union STAT_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t dummy:6;
    uint8_t BIT_OVF_IRS:1;
    uint8_t BIT_INTF:1;
  };
};

union SCLR_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t dummy:6;
    uint8_t BIT_OVS_CLR:1;
    uint8_t BIT_INT_CLR:1;
  };
};

union AVE_REG{
  uint8_t BYTE:8;
  struct{
    uint8_t dummy:2;
    uint8_t MAMOD:1;
    uint8_t dummy2:5;
  };
};

#ifdef __cplusplus
extern "C" {
#endif

void *new_grideye(I2C_HandleTypeDef *hnd);
void grideye_init(void *ge);
void grideye_get_therm(void *ge);

#ifdef __cplusplus
};
#endif


#endif /* _GRIDEYE_H_ */
