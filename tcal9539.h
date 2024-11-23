/*
 * tcal9539.h
 *
 *  Created on: Nov 10, 2024
 *      Author: nick
 */

#include <stdint.h>

#ifndef TCAL9539_H
#define TCAL9539_H

#define TCAL9539_ADDRESS 0x74  // 7-bit I2C address with A1,A0 = 0, need R/W?

#define CONFIG_PORT0_REG 0x06
#define OUTPUT_PORT0_REG 0x02

#define CONFIG_PORT1_REG 0x07
#define OUTPUT_PORT1_REG 0x03

void init_i2c();
void write_tcal9539_register(uint8_t register, uint8_t data);


#endif /* TCAL9539_H */
