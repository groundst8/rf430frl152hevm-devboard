/*
 * rom_config.c
 *
 *  Created on: Nov 9, 2024
 *      Author: nick
 */

#include <rf13m_rom_config.h>
#include <string.h>

/* Firmware System Control Byte
 *
 *     Bit 0:   ISOBlockSize                0 - 4 byte,     1 - 8 byte
 *     Bit 1:   Page                        0 - page 1,     1 - page 0 (Effective only for 4-byte block mode)
 *     Bit 2:   ROMEUSCISupportEnabled      0 - disabled,   1 - enabled (Forced to 0 on RF430FRL153H)
 *     Bit 3-5: ReservedISO
 *     Bit 6:   NFCBridgeDisable            0 - enabled,    1 - disabled (see note below)
 *     Bit 7:   ROMSensorSupportEnable      0 - disabled,   1 - enabled (Forced to 0 on RF430FRL154H)
 *
 *     NFC bridge is recommended to be disabled in this project.  Unexpected behavior can occur,
 *     trying to use it, due to the configuration being setup here.
 *
 *     If eUSCI host controller portion is needed along with the RF functionality, the default project
 *     must be used.  That is NFC cannot be supported in that application (because the I2C/SPI host controller
 *     control registers are in the same place that the NFC file needs to be).  However the rest of the FRAM
 *     memory can be used for storing and reading using ISO15693.
 */

//This project is based on the RF430FRL152H.  However it will work as well on the RF430FRL154H.
//However ROM_SENSOR_SUPPORT_DISABLED (or ROMSensorSupportEnable see above for both )must be set in the firmware system control register.  This is forced automatically on the RF430FRL154H.
//This setting is needed to disable the ROM which uses block 0... as virtual registers, however this memory is needed for NDEF purposes.
#define FIRMWARE_CONTROL_ADDRESS    0xF867
#pragma RETAIN(Firmware_System_Control_Byte);
#pragma location = FIRMWARE_CONTROL_ADDRESS
//This variable needs to be kept declared and as "volatile" for the BlockLockROM_Patched function to work properly.  Assignment can be changed however.
volatile const uint8_t Firmware_System_Control_Byte = ROM_SENSOR_SUPPORT_DISABLED + EROM_EUSCI_SUPPORT_DISABLED + NFC_BRIDGE_DISABLED + FOUR_BYTE_BLOCK + FIRST_ISO_PAGE; //0x7F,     // this value sets the firmware system control register

#pragma RETAIN(DS)
#pragma location = 0x1C00
uint8_t DS;


void rf13m_setup() {
    // ROM RF13M module setup ** The following three lines are needed for proper RF stack operation
    DS = 1;                                     // ROM variable needs to be initialized here
    asm ( " CALL #0x5CDA ");                    // Call ROM function ( Initialize function pointers)
    asm ( " CALL #0x5CAC ");                    // Call ROM function ( Check part configuration)
}

void init_iso15693(uint16_t parameters)
{

  RF13MCTL |= RF13MTXEN + RF13MRXEN + RF13MRFTOEN;  // set up rx and tx functionality on RF13M module
  // enable interrupts  ** Do not change the following two lines, needed for proper RF stack operatoin
  RF13MINT |= RF13MRXIE + RX13MRFTOIE;              // enable interrupts on RX and on timeout and over and under flow checking

  if (parameters & CLEAR_BLOCK_LOCKS )
  {
    memset ((uint8_t *) FRAM_LOCK_BLOCKS, 0xFF, FRAM_LOCK_BLOCK_AREA_SIZE);     //block is locked with a zero bit, clears FRAM and RAM lock blocks
  }

//  BlockLockAPI(3, LOCK_BLOCK);  //Test this API
//
//  BlockLockAPI(3, CHECK_LOCK);  //Test this API
}
