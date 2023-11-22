/*
 * bootloader.h
 *
 *  Created on: Nov 20, 2023
 *      Author: Ahmed Soliman
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_


// command length 1byte + command_code 1byte +  details + crc 4bytes

#define BL_DEBUG_UART			&huart2
#define BL_HOST_CMD_UART		&huart3
#define BL_CRC_ENGINE			&hcrc

#define HOST_CMD_BUFFER_SIZE			100
#define MESSAGE_DEBUG_BUFFER_SIZE		100

/*************************Commands************************/
#define CBL_GET_VER_CMD              0x10
#define CBL_GET_HELP_CMD             0x11
#define CBL_GET_CID_CMD              0x12
/* Get Read Protection Status */
#define CBL_GET_RDP_STATUS_CMD       0x13
#define CBL_GO_TO_ADDR_CMD           0x14
#define CBL_FLASH_ERASE_CMD          0x15
#define CBL_MEM_WRITE_CMD            0x16
/* Enable/Disable Write Protection */
#define CBL_ED_W_PROTECT_CMD         0x17
#define CBL_MEM_READ_CMD             0x18
/* Get Sector Read/Write Protection Status */
#define CBL_READ_SECTOR_STATUS_CMD   0x19
#define CBL_OTP_READ_CMD             0x20
/* Change Read Out Protection Level */
#define CBL_CHANGE_ROP_Level_CMD     0x21

#define VENDOR_ID				100
#define BL_SW_MAJOR_VERSION		1
#define BL_SW_MINOR_VERSION		0
#define BL_SW_PATCH_VERSION		0


#define BL_SEND_NACK	0xAB
#define BL_SEND_ACK		0xCD

#define FLASH_SECTOR2_BASE_ADDRESS	0x8008000UL

#define FLASH_SIZE	(64*1024)
#define SRAM_SIZE	(20*1024)

#define FLASH_END_ADDRESS		(FLASH_BASE+FLASH_SIZE)
#define SRAM_END_ADDRESS		(SRAM_BASE+SRAM_SIZE)


#define FLASH_ERASE_COMPLETE	0xFFFFFFFFUL
#define MAX_NUMBER_OF_PAGES		64
#define MASS_ERASE_CMD			0xFFUL

typedef void (*pvfun)(void);
typedef enum
{
	CRC_NO_MATCH = 0,
	CRC_MATCH
}CRC_VERVICATION;

typedef enum
{
	BL_NACK=0,
	BL_ACK
}BL_STATUS;

typedef enum
{
	ADDR_INVALID = 0,
	ADDR_VALID

}ADDR_VALID_CHECK;

typedef enum
{
	FLASH_ERASE_FAILED=0,
	FLASH_ERASE_SUCCEDD
}FLASH_ERASE_STATUS;

typedef enum
{
	FLASH_WRITE_FAILED=0,
	FLASH_WRITE_SUCCEDD
}FLASH_WRITE_STATUS;

typedef enum
{
	FLASH_LOCKED=0,
	FLASH_UNLOCKED
}FLASH_LOCK_STATUS;

BL_STATUS BL_Fetch_Host_CMD(void);
void BL_DEBUG_MESSAGE(char *format,...);


#endif /* INC_BOOTLOADER_H_ */
