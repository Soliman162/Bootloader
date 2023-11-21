/*
 * bootloader.c
 *
 *  Created on: Nov 20, 2023
 *      Author: Ahmed Soliman
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <memory.h>
#include "usart.h"
#include "crc.h"
#include "bootloader.h"

const uint8_t BL_CMD_ARR[12] = {
		CBL_GET_VER_CMD,
		CBL_GET_HELP_CMD,
		CBL_GET_CID_CMD,
		CBL_GET_RDP_STATUS_CMD,
		CBL_GO_TO_ADDR_CMD,
		CBL_FLASH_ERASE_CMD,
		CBL_MEM_WRITE_CMD,
		CBL_ED_W_PROTECT_CMD,
		CBL_MEM_READ_CMD,
		CBL_READ_SECTOR_STATUS_CMD,
		CBL_OTP_READ_CMD,
		CBL_CHANGE_ROP_Level_CMD
};

static inline void Bootloader_Get_Version(uint8_t *Host_Buffer);
static inline void Bootloader_Get_Help(uint8_t *Host_Buffer);
static inline void Bootloader_Get_Chip_Identification_Number(uint8_t *Host_Buffer);
static inline void Bootloader_Read_Protection_Level(uint8_t *Host_Buffer);
static inline void Bootloader_Jump_To_Address(uint8_t *Host_Buffer);
static inline void Bootloader_Erase_Flash(uint8_t *Host_Buffer);
static inline void Bootloader_Memory_Write(uint8_t *Host_Buffer);
static inline void Bootloader_Enable_RW_Protection(uint8_t *Host_Buffer);
static inline void Bootloader_Memory_Read(uint8_t *Host_Buffer);
static inline void Bootloader_Get_Sector_Protection_Status(uint8_t *Host_Buffer);
static inline void Bootloader_Read_OTP(uint8_t *Host_Buffer);
static inline void Bootloader_Change_Read_Protection_Level(uint8_t *Host_Buffer);

static CRC_VERVICATION BootLoader_CRC_verfiy(uint8_t *Data_arr,uint8_t Data_Length,uint32_t CP_host_crc);
static void BL_send_ACK(uint8_t Replay_Length);
static void BL_send_NACK(void);
static void JUMP_To_User_App(void);
static void Send_Data_To_HOST(uint8_t *pdata , uint8_t Size);
static ADDR_VALID_CHECK Address_Verfication_Check(uint32_t cp_Address);

void BL_DEBUG_MESSAGE(char *format,...)
{
	char Message[MESSAGE_DEBUG_BUFFER_SIZE]={'\0'};
	va_list args;
	va_start(args,format);
	vsprintf(Message,format,args);
	HAL_UART_Transmit(BL_DEBUG_UART,(uint8_t *)Message, sizeof(Message), HAL_MAX_DELAY);
	va_end(args);
}

BL_STATUS BL_Fetch_Host_CMD(void)
{
	BL_STATUS cmd_status = BL_ACK;
	uint8_t Cmd_Size = 0;
	uint8_t Rec_buffer[HOST_CMD_BUFFER_SIZE] = {'\0'};

	if( HAL_UART_Receive(BL_HOST_CMD_UART, (uint8_t*)Rec_buffer, 1, HAL_MAX_DELAY) != HAL_OK )
	{
		BL_DEBUG_MESSAGE("Couldn't Receive command length \r\n");
		cmd_status = BL_NACK;
	}
	else
	{
		Cmd_Size = Rec_buffer[0];
		if(HAL_UART_Receive(BL_HOST_CMD_UART, (uint8_t*)&Rec_buffer[1], Cmd_Size, HAL_MAX_DELAY) != HAL_OK)
		{
			cmd_status = BL_NACK;
			BL_DEBUG_MESSAGE("Couldn't Receive command \r\n");
		}
		else
		{
			switch (Rec_buffer[1])
			{
				case CBL_GET_VER_CMD:
					BL_DEBUG_MESSAGE("CBL_GET_VER_CMD \r\n");
					Bootloader_Get_Version(Rec_buffer);
					break;
				case CBL_GET_HELP_CMD:
					BL_DEBUG_MESSAGE("CBL_GET_HELP_CMD \r\n");
					Bootloader_Get_Help(Rec_buffer);
					break;
				case CBL_GET_CID_CMD:
					BL_DEBUG_MESSAGE("CBL_GET_CID_CMD \r\n");
					Bootloader_Get_Chip_Identification_Number(Rec_buffer);
					break;
				case CBL_GET_RDP_STATUS_CMD:
					BL_DEBUG_MESSAGE("CBL_GET_RDP_STATUS_CMD \r\n");
					Bootloader_Read_Protection_Level(Rec_buffer);
					break;
				case CBL_GO_TO_ADDR_CMD:
					BL_DEBUG_MESSAGE("CBL_GO_TO_ADDR_CMD \r\n");
					Bootloader_Jump_To_Address(Rec_buffer);
					break;
				case CBL_FLASH_ERASE_CMD:
					BL_DEBUG_MESSAGE("CBL_FLASH_ERASE_CMD \r\n");
					Bootloader_Erase_Flash(Rec_buffer);
					break;
				case CBL_MEM_WRITE_CMD:
					BL_DEBUG_MESSAGE("CBL_MEM_WRITE_CMD \r\n");
					Bootloader_Memory_Write(Rec_buffer);
					break;
				case CBL_ED_W_PROTECT_CMD:
					BL_DEBUG_MESSAGE("CBL_ED_W_PROTECT_CMD \r\n");
					Bootloader_Enable_RW_Protection(Rec_buffer);
					break;
				case CBL_MEM_READ_CMD:
					BL_DEBUG_MESSAGE("CBL_MEM_READ_CMD \r\n");
					Bootloader_Memory_Read(Rec_buffer);
					break;
				case CBL_READ_SECTOR_STATUS_CMD:
					BL_DEBUG_MESSAGE("CBL_READ_SECTOR_STATUS_CMD \r\n");
					Bootloader_Get_Sector_Protection_Status(Rec_buffer);
					break;
				case CBL_OTP_READ_CMD:
					BL_DEBUG_MESSAGE("BL_DEBUG_MESSAGE \r\n");
					Bootloader_Read_OTP(Rec_buffer);
					break;
				case CBL_CHANGE_ROP_Level_CMD:
					BL_DEBUG_MESSAGE("CBL_CHANGE_ROP_Level_CMD \r\n");
					Bootloader_Change_Read_Protection_Level(Rec_buffer);
					break;
				default:
					BL_DEBUG_MESSAGE("Invalid Command \r\n");
					break;
			}
		}
	}
	return cmd_status;
}
static void Send_Data_To_HOST(uint8_t *pdata , uint8_t Size)
{
	HAL_UART_Transmit(BL_HOST_CMD_UART, pdata, Size, HAL_MAX_DELAY);
}
CRC_VERVICATION BootLoader_CRC_verfiy(uint8_t *Data_arr,uint8_t Data_Length,uint32_t CP_host_crc)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint32_t CRC_calculated = 0;
	uint8_t Data_Counter ;
	uint32_t Data_temp = 0;

	__HAL_CRC_DR_RESET(BL_CRC_ENGINE);

	for(Data_Counter = 0; Data_Counter < Data_Length ;Data_Counter++)
	{
		Data_temp = (uint32_t)Data_arr[Data_Counter];
		CRC_calculated = HAL_CRC_Accumulate(BL_CRC_ENGINE, &Data_temp, 1);
	}

	if( CRC_calculated == CP_host_crc )
	{
		CRC_status = CRC_MATCH;
	}
	else
	{
		CRC_status = CRC_NO_MATCH;
	}
	return CRC_status;
}

static inline void BL_send_ACK(uint8_t Replay_Length)
{
	uint8_t ACK_Value[2] = {BL_SEND_ACK,Replay_Length};
	HAL_UART_Transmit(BL_HOST_CMD_UART, ACK_Value, 2, HAL_MAX_DELAY);
}
static inline void BL_send_NACK(void)
{
	uint8_t ACK_Value = BL_SEND_NACK;
	HAL_UART_Transmit(BL_HOST_CMD_UART, (uint8_t *)&ACK_Value, 1, HAL_MAX_DELAY);
}

void Bootloader_Get_Version(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint8_t BL_version[4] = {VENDOR_ID,BL_SW_MAJOR_VERSION,BL_SW_MINOR_VERSION,BL_SW_PATCH_VERSION};
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

	if( CRC_status == CRC_MATCH )
	{
		BL_send_ACK(4);
		Send_Data_To_HOST(BL_version, 4);
	}
	else
	{
		BL_send_NACK();
	}
}
void Bootloader_Get_Help(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

	if( CRC_status == CRC_MATCH )
	{
		BL_send_ACK(12);
		Send_Data_To_HOST( (uint8_t *)(&BL_CMD_ARR[0]), 12);
	}
	else
	{
		BL_send_NACK();
	}
}
void Bootloader_Get_Chip_Identification_Number(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	uint16_t MCU_ID = (uint16_t)(DBGMCU->IDCODE & 0x0000FFF);

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

	if( CRC_status == CRC_MATCH )
	{
		BL_send_ACK(2);
		Send_Data_To_HOST((uint8_t *)&MCU_ID, 2);
	}
	else
	{
		BL_send_NACK();
	}
}
void Bootloader_Read_Protection_Level(uint8_t *Host_Buffer)
{

}
void Bootloader_Jump_To_Address(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	uint32_t Jump_Addr = 0;
	ADDR_VALID_CHECK Check = ADDR_INVALID;

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

	if( CRC_status == CRC_MATCH )
	{
		BL_send_ACK(1);
		Jump_Addr = (uint32_t *)Host_Buffer[2];
		Check = Address_Verfication_Check(Jump_Addr);
		Send_Data_To_HOST((uint8_t *)&Check, 1);
		if( Check == ADDR_VALID )
		{
			BL_DEBUG_MESSAGE("Address valid 0x%X\r\n",Jump_Addr);
			pvfun address = (pvfun)(Jump_Addr+1) ;
			address();
		}else
		{
			BL_DEBUG_MESSAGE("Address not valid 0x%X\r\n",Jump_Addr);
		}
	}
	else
	{
		BL_send_NACK();
	}
}
void Bootloader_Erase_Flash(uint8_t *Host_Buffer)
{

}
void Bootloader_Memory_Write(uint8_t *Host_Buffer)
{

}
void Bootloader_Enable_RW_Protection(uint8_t *Host_Buffer)
{

}
void Bootloader_Memory_Read(uint8_t *Host_Buffer)
{

}
void Bootloader_Get_Sector_Protection_Status(uint8_t *Host_Buffer)
{

}
void Bootloader_Read_OTP(uint8_t *Host_Buffer)
{

}
void Bootloader_Change_Read_Protection_Level(uint8_t *Host_Buffer)
{

}

void JUMP_To_User_App(void)
{
	uint32_t APP_MSP = *((volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS);
	uint32_t APP_Entry_Point = *(( volatile uint32_t *)(APP_MSP+4));
	pvfun Reset_Handler = (pvfun)APP_Entry_Point;
	__set_MSP(APP_MSP);
	HAL_DeInit();
	HAL_RCC_DeInit();
	Reset_Handler();
}

ADDR_VALID_CHECK Address_Verfication_Check(uint32_t cp_Address)
{
	ADDR_VALID_CHECK Check_addr = ADDR_INVALID;
	if( (cp_Address >= FLASH_BASE) && (cp_Address <= FLASH_END_ADDRESS) )
	{
		Check_addr = ADDR_VALID;
	}
	else if( (cp_Address >= SRAM_BASE) && (cp_Address <= SRAM_END_ADDRESS) )
	{
		Check_addr = ADDR_VALID;
	}

	return Check_addr;
}
