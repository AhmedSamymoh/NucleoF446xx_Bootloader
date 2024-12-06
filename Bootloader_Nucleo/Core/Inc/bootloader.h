/**
 * @file     bootloader.h
 * @date     Nov 22, 2024
 * @author   Ahmed Samy
 * @brief    A Prototype Bootloader Implementation
 * @version info : Platform : stm32f446re, IDE : STM32CubeIDE, Compiler : Arm Compiler 6
 *
 * @copyright Alexandria University Copyright (c) 2024
 *
 */


#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_


/********************************************** Section : Includes ********************************************/


#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "main.h"
#include "Std_Types.h"

/**************************************** Section: Data Type Declarations **************************************/


typedef enum{
	BL_NOT_ACK=0x7F,
	BL_ACK=0xA5
} BL_Status_t;

typedef enum{
	Address_INVALID=0,
	Address_VALID
} BL_Address_Status_t;

typedef enum{
	FLASH_PAYLOAD_WRITE_FAILED=0x00,
	FLASH_PAYLOAD_WRITE_PASSED
} BL_FlashPayload_Status_t;

typedef enum{
	SECTOR_W_PROTECTION=0,
	SECTOR_WR_PROTECTION
} BL_Sector_Protection_Mode_t;

typedef enum{
	ERASE_FLAG_UNFOUND,
	ERASE_FLAG_FOUND
} BL_EraseFlag_t;

typedef void (*pMainApp)(void);

/****************************************** Section: Macro Declarations ****************************************/


/*
* Supported Commands
*/
#define BL_CMD_GET_VER                 0x51
#define BL_CMD_GET_HELP                0x52
#define BL_CMD_GET_CHIP_ID             0x53
#define BL_CMD_GET_RDP_STATUS          0x54
#define BL_CMD_GO_TO_ADDR              0x55
#define BL_CMD_FLASH_ERASE             0x56
#define BL_CMD_MEM_WRITE               0x57
#define BL_CMD_ENABLE_RW_PROTECT       0x58
#define BL_CMD_MEM_READ                0x59
#define BL_CMD_READ_SECTOR_STATUS      0x5A
#define BL_CMD_OTP_READ                0x5B
#define BL_CMD_DISABLE_RW_PROTECT      0x5C


/*
* CRC Verification
*/
#define CRC_VERIFING_FAILED			   0x00
#define CRC_VERIFING_PASS			   0x01

/*
* BL_Acknolagement
*/
#define SEND_NOT_ACK                   0xAB
#define SEND_ACK                       0xCD

/*
* BL_VersionInfo
*/
#define BL_SW_vendor_ID        		   1u
#define BL_SW_major_version    		   2u
#define BL_SW_minor_version    		   2u
#define BL_SW_patch_version    		   1u

/*
* BL FLAG
*/
#define ERASE_FLAG_ADDR    			   0x0807FFF0
#define ERASE_FLAG_SET     			   0x74747474
#define ERASE_FLAG_CLEARED 			   0xFFFFFFFF

/*
* to Get ChipID
*/
#define DBG_MCU_ID_REG				   (*((volatile uint32*) 0xE0042000))

/*
*  RDP: Read protection option byte.
*/
#define RDP_USER_OPTION_WORD           (*((volatile uint32*) 0x1FFFC000))


#define HOST_MAX_SIZE				   200


#define INVALID_PAGE_NUMBER			   0x00
#define VALID_PAGE_NUMBER			   0x01
#define UNSUCCESSFUL_ERASE			   0x02
#define SUCCESSFUL_ERASE			   0x03

#define HAL_SUCCESSFUL_ERASE		   0xFFFFFFFFU

#define BL_FLASH_MAX_PAGE_NUMBER	   16
#define BL_FLASH_MASS_ERASE			   0xFF

#define STM32F446xx_FLASH_BASE	       0x08000000UL
#define STM32F446xx_FLASH_END	       0x0807FFFFUL
#define STM32F446xx_SRAM1_BASE		   0x20000000UL
#define STM32F446xx_SRAM1_END		   ( STM32F446xx_SRAM1_BASE +(128*1024) )



/*
#define STM32F103_SRAM_SIZE 		   (20 * 1024)
#define STM32F103_SRAM_END  		   (SRAM_BASE + STM32F103_SRAM_SIZE)
#define STM32F103_FLASH_SIZE 		   (64 * 1024)
#define STM32F103_FLASH_END  		   (FLASH_BASE + STM32F103_FLASH_SIZE)
#define FLASH_SECTOR2_BASE_ADDRESS      0x08008000U
*/

/************************************* Section : Global Variables Definitions **********************************/
extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;

/************************************* Section : Macro Functions Definitions ***********************************/

/**************************************** Section : Functions Declarations *************************************/


void BL_voidHandle_GetVersion_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_GetHelp_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_GetChipID_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_GetRDPStatus_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_GoToAddress_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_FlashErase_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_MemoryWrite_CMD(uint8 * copy_pu8CmdPacket);

void BL_voidHandle_EnableRWProtect_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_DisableRWProtect_CMD(uint8 * copy_pu8CmdPacket);

void BL_voidHandle_MemoryRead_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_ReadSectorStatus_CMD(uint8 * copy_pu8CmdPacket);
void BL_voidHandle_OTPRead_CMD(uint8 * copy_pu8CmdPacket);


#endif /* INC_BOOTLOADER_H_ */
