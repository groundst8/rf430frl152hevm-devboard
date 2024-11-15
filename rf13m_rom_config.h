/*
 * rom_config.h
 *
 *  Created on: Nov 9, 2024
 *      Author: nick
 */
#include <stdint.h>
#include <rf430frl152h.h>

#ifndef ROM_CONFIG_H
#define ROM_CONFIG_H

extern uint8_t DS;

#define CLEAR_BLOCK_LOCKS                               BIT3
#define FRAM_LOCK_BLOCK_AREA_SIZE                       38
#define FRAM_LOCK_BLOCKS                                0xF840  //Address of ISO15693 lock blocks


#define ROM_EUSCI_SUPPORT_ENABLED       BIT2
#define EROM_EUSCI_SUPPORT_DISABLED     0
#define ROM_SENSOR_SUPPORT_ENABLED      BIT7
#define ROM_SENSOR_SUPPORT_DISABLED     0
#define NFC_BRIDGE_DISABLED             BIT6
#define NFC_BRIDGE_ENABLED              0
#define EIGHT_BYTE_BLOCK                BIT0
#define FOUR_BYTE_BLOCK_MASK            BIT0
#define FOUR_BYTE_BLOCK                 0
#define FIRST_ISO_PAGE_MASK             BIT1
#define FIRST_ISO_PAGE                  BIT1
#define SECOND_ISO_PAGE                 0
#define FRAM_BLOCKS_8                   0xF3


#define CHECK_LOCK                  1
#define LOCK_BLOCK                  0
#define LOCKED_FLAG                 BIT0

void rf13m_setup();
void init_iso15693(uint16_t parameters);


#endif /* ROM_CONFIG_H */
