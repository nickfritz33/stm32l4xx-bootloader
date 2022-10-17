/**
  ******************************************************************************
  * STM32 Bootloader
  * This is inspired by : https://github.com/akospasztor/stm32-bootloader
  ******************************************************************************
  * @authors Akos Pasztor, Francois Rainville
  * @initial file   bootloader.h
  * @file 	btld.h
  * @brief  Bootloader header
  *	        This file contains the bootloader configuration parameters,
  *	        function prototypes and other required macros and definitions.
  * @see    Please refer to README for detailed information.
  ******************************************************************************
  * Copyright (c) 2018 Akos Pasztor.                    https://akospasztor.com
  * Copyright (c) 2018 Fran�ois Rainville.
  ******************************************************************************
**/

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include <stdint.h>

/*** Bootloader Configuration *************************************************/
#define SET_VECTOR_TABLE        1       /* Automatically set vector table location before launching application */
#define CLEAR_RESET_FLAGS       1       /* If enabled: bootloader clears reset flags. (This occurs only when OBL RST flag is active.)
                                           If disabled: bootloader does not clear reset flags, not even when OBL RST is active. */
#define BL_ADDRESS		(uint32_t)0x08000000	/* Start address of bootloader */
#define BL_SIZE			(uint32_t)0x00040000	/* Length of the bootloader (256KB) */

#define APP_ADDRESS     (uint32_t)0x08040000    /* Start address of application space in flash */
#define END_ADDRESS     (uint32_t)0x0807FFFF    /* End address of application space (addr. of last byte) */
#define CRC_ADDRESS     (uint32_t)0x080C0000    /* Start address of application checksum in flash (not including golden image) */
#define SYSMEM_ADDRESS  (uint32_t)0x1FFF0000    /* Address of System Memory (ST Bootloader) */
#define GI_APP_ADDRESS  (uint32_t)0x08080000    /* Start address of golden image application space in flash */
#define GI_END_ADDRESS  (uint32_t)0x080BFFFF    /* End address of application space (addr. of last byte) */

/* MCU RAM size, used for checking accurately whether flash contains valid application */
#define RAM_SIZE        (uint32_t)0x00050000	// 320 kB
#define FLASH_SIZE		(uint32_t)0x00100000	// 1 MB


#define CANRX_SA 0x00
#define CANTX_SA 0x01

//#define RX_CMD_CANID 0x00FF00
//#define TX_HEARTBEAT_CANID 0x00FF00
//#define TX_FEEDBACK_CANID 0x00FE00
#define STDID_CANID 0x007E

//#define RXFILTERMASK 0xFFFF00FF
//#define RXFILTERID 0x0000FF00 + CANRX_SA

/******************************************************************************/

/* Defines -------------------------------------------------------------------*/
#define FLASH_PAGE_NBPERBANK    64             /* Number of pages per bank in flash */
#define APP_SIZE	(uint32_t)(((END_ADDRESS - APP_ADDRESS) + 1)) /* Size of application in DWORD (32bits or 4bytes) */

/* Bootloader Error Codes */
enum
{
    BL_OK = 0,
    BL_NO_APP,
    BL_SIZE_ERROR,
    BL_CHKS_ERROR,
    BL_ERASE_ERROR,
    BL_WRITE_ERROR,
    BL_OBP_ERROR
};

/* Flash Protection Types */
enum
{
    BL_PROTECTION_NONE  = 0,
    BL_PROTECTION_WRP   = 0x1,
    BL_PROTECTION_RDP   = 0x2,
    BL_PROTECTION_PCROP = 0x4,
};

/* Functions -----------------------------------------------------------------*/
void    Bootloader_Init(void);
uint8_t btld_EraseFlash(void);

void    btld_FlashBegin(void);
uint8_t btld_FlashNext_32(uint32_t data,uint32_t* index);
uint8_t btld_FlashNext(uint64_t data);
void    btld_FlashEnd(void);
uint8_t btld_CopyFlash(void);

uint8_t btld_GetProtectionStatus(void);
uint8_t btld_ConfigProtection(uint32_t protection);

uint8_t btld_CheckSize(uint32_t appsize);
uint32_t btld_GetGIChecksum(void);
uint32_t btld_GetBootChecksum(void);
uint32_t btld_GetChecksum(void);
uint8_t btld_SaveChecksum(void);
uint8_t btld_CheckForApplication(void);

void    btld_JumpToApp(void);
void    btld_JumpToGoldenImage(void);
void    btld_JumpToSysMem(void);

#endif /* __BOOTLOADER_H */
