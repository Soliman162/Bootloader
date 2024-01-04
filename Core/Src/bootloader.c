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
#include "gpio.h"
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

static inline void Bootloader_Change_Protection_Level(uint8_t *Host_Buffer);

static void Test_jump(void);

static CRC_VERVICATION BootLoader_CRC_verfiy(uint8_t *Data_arr,uint8_t Data_Length,uint32_t CP_host_crc);
static void BL_send_ACK(uint8_t Replay_Length);
static void BL_send_NACK(void);
static void JUMP_To_User_App(void);
static void Send_Data_To_HOST(uint8_t *pdata , uint8_t Size);
static ADDR_VALID_CHECK Address_Verfication_Check(uint32_t cp_Address);
static FLASH_ERASE_STATUS Perform_Flash_Erase(uint8_t start_page , uint8_t Number_ofPages);
static FLASH_WRITE_STATUS Perform_Flash_Write(uint8_t *Host_payload,uint32_t Payload_start_address,uint16_t Payload_Length);
static uint8_t GET_Flash_protection_Level(void);
static FLASH_change_Protaction_STATUS Change_Read_Level(uint32_t cp_RDP_level);

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
	uint8_t Cmd_Size = 0;
	BL_STATUS cmd_status = BL_ACK;
	uint8_t Rec_buffer[HOST_CMD_BUFFER_SIZE] = {'\0'};

	if( HAL_UART_Receive(BL_HOST_CMD_UART, (uint8_t*)Rec_buffer, 1, HAL_MAX_DELAY) != HAL_OK )
	{
		cmd_status = BL_NACK;
	}
	else
	{
		Cmd_Size = Rec_buffer[0];
		if(HAL_UART_Receive(BL_HOST_CMD_UART, (uint8_t*)&Rec_buffer[1], Cmd_Size, HAL_MAX_DELAY) != HAL_OK)
		{
			cmd_status = BL_NACK;
		}
		else
		{
			switch (Rec_buffer[1])
			{
				case CBL_GET_VER_CMD: Bootloader_Get_Version(Rec_buffer); break;	// ok
				case CBL_GET_HELP_CMD: Bootloader_Get_Help(Rec_buffer); break;		// ok
				case CBL_GET_CID_CMD: Bootloader_Get_Chip_Identification_Number(Rec_buffer); break;	// ok
				case CBL_GET_RDP_STATUS_CMD: Bootloader_Read_Protection_Level(Rec_buffer); break;	// ok
				case CBL_GO_TO_ADDR_CMD: Bootloader_Jump_To_Address(Rec_buffer); break;
				case CBL_FLASH_ERASE_CMD: Bootloader_Erase_Flash(Rec_buffer); break; // ok
				case CBL_MEM_WRITE_CMD: Bootloader_Memory_Write(Rec_buffer); break;	 //
				case CBL_CHANGE_ROP_Level_CMD: Bootloader_Change_Protection_Level(Rec_buffer); break; // ok
				default:
#if DEBUG_MSG_FLAG == 1
					BL_DEBUG_MESSAGE("Invalid Command \r\n");
#endif
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

#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("CBL_GET_VER_CMD \r\n");
#endif
	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		BL_send_ACK(4);
		Send_Data_To_HOST(BL_version, 4);
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}
void Bootloader_Get_Help(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("CBL_GET_HELP_CMD \r\n");
#endif
	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		BL_send_ACK(12);
		Send_Data_To_HOST( (uint8_t *)(&BL_CMD_ARR[0]), 12);
		//JUMP_To_User_App();/******************************************************/
		Test_jump();
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}

void Bootloader_Get_Chip_Identification_Number(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	uint16_t MCU_ID = 0 ;

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("CBL_GET_CID_CMD \r\n");
#endif
	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		MCU_ID = (uint16_t)(DBGMCU->IDCODE & 0x00000FFF);
		BL_send_ACK(2);
		Send_Data_To_HOST((uint8_t*)&MCU_ID, 2);
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}

static uint8_t GET_Flash_protection_Level(void)
{
	static FLASH_OBProgramInitTypeDef OBProgram;
	HAL_FLASHEx_OBGetConfig(&OBProgram);
	return (uint8_t)(OBProgram.RDPLevel);
}

void Bootloader_Read_Protection_Level(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	uint8_t RPD_level = 0xEE;

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("CBL_GET_RDP_STATUS_CMD \r\n");
#endif
	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		BL_send_ACK(1);
		RPD_level = GET_Flash_protection_Level();
		Send_Data_To_HOST((uint8_t *)&RPD_level, 1);
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}

void Bootloader_Jump_To_Address(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	uint32_t Jump_Addr = 0;
	ADDR_VALID_CHECK Check = ADDR_INVALID;

	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("CBL_GO_TO_ADDR_CMD \r\n");
#endif
	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		BL_send_ACK(1);
		Jump_Addr = *((uint32_t *)&Host_Buffer[2]);
		Check = Address_Verfication_Check(Jump_Addr);
		Send_Data_To_HOST((uint8_t *)&Check, 1);
		if( Check == ADDR_VALID )
		{
#if DEBUG_MSG_FLAG == 1
			BL_DEBUG_MESSAGE("Address valid 0x%X\r\n",Jump_Addr);
#endif
			pvfun address = (pvfun)(Jump_Addr+1) ;
			address();
		}else
		{
#if DEBUG_MSG_FLAG == 1
			BL_DEBUG_MESSAGE("Address not valid 0x%X\r\n",Jump_Addr);
#endif
		}
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}

FLASH_ERASE_STATUS Perform_Flash_Erase(uint8_t start_page , uint8_t Number_ofPages)
{
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef Flash_Config;
	HAL_StatusTypeDef Erase_check = HAL_OK;
	FLASH_ERASE_STATUS status_Check = FLASH_ERASE_SUCCEDD;

	Erase_check = HAL_FLASH_Unlock();
	if( Erase_check == HAL_OK )
	{
		Flash_Config.TypeErase = FLASH_TYPEERASE_PAGES;
		Flash_Config.Banks = FLASH_BANK_1 ;
		if( start_page == MASS_ERASE_CMD )
		{
			Flash_Config.NbPages = 50;//MAX_NUMBER_OF_PAGES - start_page;
			Flash_Config.PageAddress = (uint32_t)FLASH_SECTOR2_BASE_ADDRESS; //(uint32_t)(FLASH_BASE+(start_page*1024));
		}
		else if( (start_page+Number_ofPages) <= MAX_NUMBER_OF_PAGES )
		{
			Flash_Config.NbPages = Number_ofPages;
			Flash_Config.PageAddress = (uint32_t)(FLASH_SECTOR2_BASE_ADDRESS+(start_page*1024));
		}
		else if( (start_page+Number_ofPages) > MAX_NUMBER_OF_PAGES )
		{
			Number_ofPages = MAX_NUMBER_OF_PAGES - start_page;
			Flash_Config.NbPages = Number_ofPages;
			Flash_Config.PageAddress = (uint32_t)(FLASH_SECTOR2_BASE_ADDRESS + (start_page*1024));
		}
		Erase_check = HAL_FLASHEx_Erase(&Flash_Config, &PageError);
		if( (Erase_check != HAL_OK) ||
			(PageError != FLASH_ERASE_COMPLETE )
		   )
		{
			status_Check = FLASH_ERASE_FAILED;
		}
		HAL_FLASH_Lock();
	}
	else
	{
		status_Check = FLASH_ERASE_FAILED;
	}
	return status_Check;
}
void Bootloader_Erase_Flash(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	FLASH_ERASE_STATUS status_Check = FLASH_ERASE_FAILED;
	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("CBL_FLASH_ERASE_CMD \r\n");
#endif
	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		status_Check = Perform_Flash_Erase( *((uint32_t *)&Host_Buffer[2]) ,Host_Buffer[3]);
		BL_send_ACK(1);
		Send_Data_To_HOST((uint8_t *)&status_Check, 1);
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}
FLASH_WRITE_STATUS Perform_Flash_Write(uint8_t *Host_payload,uint32_t Payload_start_address,uint16_t Payload_Length)
{
	HAL_StatusTypeDef Flash_lock_Check = HAL_ERROR;
	HAL_StatusTypeDef Flash_Program_Write_Check = HAL_ERROR ;
	FLASH_WRITE_STATUS Flash_write_check = FLASH_WRITE_SUCCEDD;
	ADDR_VALID_CHECK Address_check = ADDR_INVALID;
	uint8_t Page_Counter = 0;

	Flash_lock_Check = HAL_FLASH_Unlock();
	if( Flash_lock_Check == HAL_OK )
	{
		Address_check = Address_Verfication_Check(Payload_start_address);
		if( Address_check == ADDR_VALID )
		{
			for(Page_Counter=0;Page_Counter<=(Payload_Length-4);Page_Counter+=4)
			{
				Flash_Program_Write_Check = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(Payload_start_address+Page_Counter), *((uint32_t *)&Host_payload[Page_Counter]));
				if( Flash_Program_Write_Check != HAL_OK )
				{
					Flash_write_check = FLASH_WRITE_FAILED;
					break;
				}
			}
		}
		else
		{
			Flash_write_check = FLASH_WRITE_FAILED;
		}
		HAL_FLASH_Lock();
	}
	else
	{
		Flash_write_check = FLASH_WRITE_FAILED;
	}
	return Flash_write_check;
}

void Bootloader_Memory_Write(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	FLASH_WRITE_STATUS status_Check = FLASH_WRITE_FAILED;
	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);

#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("CBL_MEM_WRITE_CMD \r\n");
#endif

	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		BL_send_ACK(1);
		status_Check = Perform_Flash_Write((uint8_t *)&Host_Buffer[7],*((uint32_t *)&Host_Buffer[2]),Host_Buffer[6]);
		Send_Data_To_HOST((uint8_t *)&status_Check, 1);
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
	BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}
//void Bootloader_Enable_RW_Protection(uint8_t *Host_Buffer)
//{
//
//}
//void Bootloader_Memory_Read(uint8_t *Host_Buffer)
//{
//
//}
//void Bootloader_Get_Sector_Protection_Status(uint8_t *Host_Buffer)
//{
//
//}
//void Bootloader_Read_OTP(uint8_t *Host_Buffer)
//{
//
//}
FLASH_change_Protaction_STATUS Change_Read_Level(uint32_t cp_RDP_level)
{
	FLASH_change_Protaction_STATUS Flash_protaction_level = CHANGE_READ_LEVEL_SUCCEDD;
	HAL_StatusTypeDef Lock_unlock_check = HAL_ERROR;
	FLASH_OBProgramInitTypeDef  FLASH_OBProgram;

	Lock_unlock_check = HAL_FLASH_Unlock();
	if( Lock_unlock_check == HAL_OK )
	{
		Lock_unlock_check = HAL_FLASH_OB_Unlock();
		if( Lock_unlock_check == HAL_OK )
		{
			FLASH_OBProgram.OptionType = OPTIONBYTE_RDP;
			FLASH_OBProgram.WRPState = OB_WRPSTATE_ENABLE;
			FLASH_OBProgram.WRPPage = OB_WRP_ALLPAGES;
			FLASH_OBProgram.Banks = FLASH_BANK_1;

			if( cp_RDP_level == 1 )
			{
				FLASH_OBProgram.RDPLevel = OB_RDP_LEVEL_1;
			}
			else
			{
				FLASH_OBProgram.RDPLevel = OB_RDP_LEVEL_0;
			}
			Lock_unlock_check = HAL_FLASHEx_OBProgram(&FLASH_OBProgram);
			if( Lock_unlock_check == HAL_OK )
			{
				HAL_FLASH_OB_Launch();
			}
			else
			{
				Flash_protaction_level = CHANGE_READ_LEVEL_FAILED;
			}
			HAL_FLASH_OB_Lock();
			HAL_FLASH_Lock();
		}
		else
		{
			Flash_protaction_level = CHANGE_READ_LEVEL_FAILED;
			HAL_FLASH_Lock();
		}
	}
	else
	{
		Flash_protaction_level = CHANGE_READ_LEVEL_FAILED;
#if DEBUG_MSG_FLAG == 1
		BL_DEBUG_MESSAGE("Couldn't unlock Flash\r\n");
#endif

	}
	return Flash_protaction_level;
}
void Bootloader_Change_Protection_Level(uint8_t *Host_Buffer)
{
	CRC_VERVICATION CRC_status = CRC_MATCH;
	uint16_t Pcaket_length = Host_Buffer[0] + 1;
	uint32_t Host_CRC = *((uint32_t *)(Host_Buffer+(Pcaket_length-4)));
	CRC_status = BootLoader_CRC_verfiy(Host_Buffer,Pcaket_length-4,Host_CRC);
	FLASH_change_Protaction_STATUS Flash_protaction_level = CHANGE_READ_LEVEL_SUCCEDD;

	if( CRC_status == CRC_MATCH )
	{
#if DEBUG_MSG_FLAG == 1
		BL_DEBUG_MESSAGE("send ACK \r\n");
#endif
		BL_send_ACK(1);
		Flash_protaction_level = Change_Read_Level((uint32_t)Host_Buffer[2]);
		Send_Data_To_HOST((uint8_t *)&Flash_protaction_level, 1);
	}
	else
	{
#if DEBUG_MSG_FLAG == 1
		BL_DEBUG_MESSAGE("send NACK \r\n");
#endif
		BL_send_NACK();
	}
}

void JUMP_To_User_App(void)
{
	uint32_t APP_MSP = *((volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS);
	uint32_t APP_Entry_Point = *(( volatile uint32_t *)(APP_MSP+4));
	pvfun App_Reset_Handler = (pvfun)APP_Entry_Point;
	__set_MSP(APP_MSP);
	HAL_DeInit();
	HAL_RCC_DeInit();
	App_Reset_Handler();
}

ADDR_VALID_CHECK Address_Verfication_Check(uint32_t cp_Address)
{
	ADDR_VALID_CHECK Check_addr = ADDR_INVALID;
	if( ((cp_Address >= FLASH_BASE) && (cp_Address <= FLASH_END_ADDRESS)) ||
		((cp_Address >= SRAM_BASE) && (cp_Address <= SRAM_END_ADDRESS))
	  )
	{
		Check_addr = ADDR_VALID;
	}
	return Check_addr;
}

static void Test_jump(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}

