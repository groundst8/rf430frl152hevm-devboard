/*
 * rf13m_rom_patch.h
 *
 *  Created on: Nov 9, 2024
 *      Author: nick
 */

#ifndef RF13M_ROM_PATCH_H
#define RF13M_ROM_PATCH_H

typedef void(*DriverFunction)(void);


#define CRC_LENGTH_IN_BUFFER          2  // the CRC bytes take 2 bytes in the packet
#define DATA_IN_LENGTH                1  // only 1 byte of data in expected


#define DRIVER_TABLE_START              0xFFCE                  // starting address for driver table
#define DRIVER_TABLE_KEY                0xCECE                  // identifier indicating start and end of driver table
#define BLOCK_LOCK_ID                   0x2600                  // Block Lock Code for ROM
#define GET_SYSTEM_INFO_ID              0x002B                  // Get System Info ISO15693 command ID
#define GET_MUL_BLCK_SEC_STATUS_ID      0x002C                  // Get Multiple Block Security Status ISO15693 command ID
#define USER_CUSTOM_COMMAND_ID          0x00AA                  // user custom command, range from A0 - D0

#define NUMBER_OF_DRIVER_FUNCTIONS      4                       // the amount of patched functions
//------------------------------------------------------------------------------
#define CUSTOM_COMMAND         (DRIVER_TABLE_START-2)
#define CUSTOM_COMMAND_ADDR    (DRIVER_TABLE_START-4)

#define GET_SYSTEM_INFO_COMMAND (DRIVER_TABLE_START-6)                  // DIGITAL_SENSOR_DRIVER_ID, see below
#define GET_SYSTEM_INFO_ADDR    (DRIVER_TABLE_START-8)

#define GET_MULTIPLE_BLOCK_SECURITY_STATUS_COMMAND (DRIVER_TABLE_START-10)                      // INIT_DIGITAL_SENSOR_DRIVER_ID, see below
#define GET_MULTIPLE_BLOCK_SECURITY_STATUS_ADDR    (DRIVER_TABLE_START-12)

#define BLOCK_LOCK_ROM_COMMAND (DRIVER_TABLE_START-14)                      // INIT_DIGITAL_SENSOR_DRIVER_ID, see below
#define BLOCK_LOCK_ROM_ADDR    (DRIVER_TABLE_START-16)

#define DRIVER_TABLE_END  (DRIVER_TABLE_START-2-(NUMBER_OF_DRIVER_FUNCTIONS*4))


#endif /* RF13M_ROM_PATCH_H */
