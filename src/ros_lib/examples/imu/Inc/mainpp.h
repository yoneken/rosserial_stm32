/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

void setup(void);
void loop(void);

void SPI_Mem_Write(unsigned short int chip, uint8_t address, short num, const uint8_t data[]);
void SPI_Mem_Read(unsigned short int chip, uint8_t address, void (*callback)(uint8_t dat[]), short num);


#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
