/*
 * @Author: xmprocat
 * @Date: 2022-07-31 15:36:14
 * @LastEditors: xmprocat
 * @LastEditTime: 2022-08-06 21:24:03
 * @Description: 
 */
#ifndef __INA3221_2_H
#define __INA3221_2_H
#include "stdint.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define INA3221_2_ADDR1 0x80  // A0=GND
#define INA3221_2_ADDR2 0x82  // A0=VS
#define INA3221_2_ADDR3 0x84  // A0=SDA
#define INA3221_2_ADDR4 0x86  // A0=SCL

void INA3221_2_Init(uint8_t addr);
uint16_t INA3221_2_GetVoltage(uint8_t addr, uint8_t channel);
int INA3221_2_GetShuntVoltage(uint8_t addr, uint8_t channel);
int INA3221_2_Median_filtering1();
int INA3221_2_Median_filtering2();
int INA3221_2_Median_filtering3();
#endif
