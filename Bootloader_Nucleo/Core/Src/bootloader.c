/**
 * @file     bootloader.c
 * @date     Nov 22, 2024
 * @author   Ahmed Samy
 * @brief    A Prototype Bootloader APIs Implementation
 *
 * @copyright Alexandria University Copyright (c) 2025
 *
 */

/********************************************** Section : Includes ********************************************/

#include "bootloader.h"



/****************************************************************************************************************/
/******************************************* Section : Local Functions ******************************************/
/****************************************************************************************************************/

static uint8 u8VerifyCRC(uint8 * copy_pu8DataArr, uint8 copy_u8Length, uint32 copy_u32HostCRC);
static BL_Address_Status_t ValidateAddress(uint32 u32Address);
static void voidSendAck(uint8 copy_u8ReplyLength);
static void voidSendNotAck(void);
static HAL_StatusTypeDef u8Execute_FlashErase(uint8 copy_u8SectorNumber, uint8 copy_u8NumberOfSectors);
static uint8 u8Execute_MemoryWrite(uint8 * Copy_pu8Buffer, uint32 Copy_u32Address, uint8 Copy_u8Length);
static void voidSet_EraseFlag(void);
static BL_EraseFlag_t VerfiyErase_Flag(void);

/**************************************** Section : Functions Definition ***************************************/

/**
 * @brief Handles the GetVersion command to send the bootloader version to the host.
 * 
 * @param copy_pu8CmdPacket Pointer to the command packet containing:
 *        - [0]: Command length
 *        - [1]: Command identifier
 *        - [2-5]: CRC (4 bytes)
 */
void BL_voidHandle_GetVersion_CMD(uint8 * copy_pu8CmdPacket){
	uint8 BL_version[4] = {BL_SW_vendor_ID, BL_SW_major_version, BL_SW_minor_version ,BL_SW_patch_version};
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ; 
	
	CmdLen = copy_pu8CmdPacket[0]+1;

	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	if(CRCStatus == CRC_VERIFING_PASS){
		voidSendAck(4);
		HAL_UART_Transmit(&huart2,(uint8 *)BL_version,4, HAL_MAX_DELAY);
	}else{
		voidSendNotAck();
	}
}

/**
 * @brief Handles the GetHelp command to send a list of supported bootloader commands to the host.
 * 
 * @param copy_pu8CmdPacket Pointer to the command packet containing:
 *        - [0]: Command length
 *        - [1]: Command identifier
 *        - [2-5]: CRC (4 bytes)
 * 
 */
void BL_voidHandle_GetHelp_CMD(uint8 * copy_pu8CmdPacket){
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ;

	CmdLen = copy_pu8CmdPacket[0]+1;
	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	if(CRCStatus == CRC_VERIFING_PASS){

		uint8 BL_Supported_CMDs[] = {
				BL_CMD_GET_VER            ,
				BL_CMD_GET_HELP           ,
				BL_CMD_GET_CHIP_ID        ,
				BL_CMD_GET_RDP_STATUS     ,
				BL_CMD_GO_TO_ADDR         ,
				BL_CMD_FLASH_ERASE        ,
				BL_CMD_MEM_WRITE          ,
				BL_CMD_ENABLE_RW_PROTECT  ,
				BL_CMD_MEM_READ           ,
				BL_CMD_READ_SECTOR_STATUS ,
				BL_CMD_OTP_READ           ,
				BL_CMD_DISABLE_RW_PROTECT
		};

		voidSendAck(sizeof(BL_Supported_CMDs));
		HAL_UART_Transmit(&huart2,(uint8 *)BL_Supported_CMDs,sizeof(BL_Supported_CMDs), HAL_MAX_DELAY);
	}else{
		voidSendNotAck();
	}

}

/**
 * @brief Handles the GetChipID command to retrieve the unique device identifier (Device ID).
 * 
 * @param copy_pu8CmdPacket Pointer to the command packet containing:
 *        - [0]: Command length
 *        - [1]: Command identifier
 *        - [2-5]: CRC (4 bytes)
 */
void BL_voidHandle_GetChipID_CMD(uint8 * copy_pu8CmdPacket){
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ;
	uint16 DeviceID;

	CmdLen = copy_pu8CmdPacket[0]+1;

	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	if(CRCStatus == CRC_VERIFING_PASS){
		/* DEV_ID[11:0]: Device identifier*/
		DeviceID = (DBG_MCU_ID_REG & 0x0fff);

		voidSendAck(2);
		HAL_UART_Transmit(&huart2,(uint8 *)&DeviceID,2, HAL_MAX_DELAY);
	}else{
		voidSendNotAck();
	}
}


/**
 * @brief Handles the "Get RDP Status" command, which retrieves the current Readout Protection (RDP) status.
 * 
 * @param copy_pu8CmdPacket Pointer to the command packet containing:
 *        - [0]: Command length
 *        - [1]: Command identifier
 *        - [2-5]: RDP user option word (for checking status)
 *        - [6-9]: CRC (4 bytes)
 */
void BL_voidHandle_GetRDPStatus_CMD(uint8 * copy_pu8CmdPacket){
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ;
	uint8 RDP_Status;

	CmdLen = copy_pu8CmdPacket[0]+1;

	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	if(CRCStatus == CRC_VERIFING_PASS){
		RDP_Status = (uint8)((RDP_USER_OPTION_WORD >> 8) & 0xff );
		voidSendAck(1);
		HAL_UART_Transmit(&huart2,&RDP_Status,1, HAL_MAX_DELAY);
	}else{
		voidSendNotAck();
	}
}


/**
 * @brief Handles the "Go To Address" bootloader command, which jumps to a specific address provided by the host.
 *
 * @param copy_pu8CmdPacket Pointer to the command packet containing:
 *   - [0]: Command length
 *   - [1]: Command identifier
 *   - [2-5]: Target address to jump to (4 bytes)
 *   - [6-9]: CRC (4 bytes)
 */
void BL_voidHandle_GoToAddress_CMD(uint8 * copy_pu8CmdPacket){
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ;
	uint32 HOST_Address;
	BL_Address_Status_t Address_Status;

	CmdLen = copy_pu8CmdPacket[0]+1;

	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	if(CRCStatus == CRC_VERIFING_PASS){
		/*reading address given by the host*/
		HOST_Address = *((uint32 *)(&copy_pu8CmdPacket[2]));
		Address_Status = ValidateAddress(HOST_Address);
		if(Address_Status == Address_VALID){
			voidSendAck(1);
			HAL_UART_Transmit(&huart2,&Address_Status,1, HAL_MAX_DELAY);

			/*Define a pointer to function*/
			void (*pvFuncPtr)(void) = NULL;
			/*increment address by 1 to make t-bit = 1
			 * as t-bit (Thumb-bit) to assure operating in Thumb state */
			HOST_Address ++;

			pvFuncPtr = (void *)HOST_Address;

			/*Jump to the given address*/
			pvFuncPtr();

		}else{
			voidSendNotAck();
		}
	}else{
		voidSendNotAck();
	}
}

/**
 * @brief Handles the flash erase command, validating CRC, initiating sector erase, 
 *        and verifying the erase flag.
 * 
 * @param copy_pu8CmdPacket Pointer to the command packet containing:
 *  	  - [0]: Command length
 *        - [1]: Command identifier
 *        - [2]: Sector number (1 byte)
 *        - [3]: Number of sectors (1 byte)
 *        - [4-8]: CRC (4 bytes)
 * 
 */
void BL_voidHandle_FlashErase_CMD(uint8 * copy_pu8CmdPacket){
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ;
	HAL_StatusTypeDef EraseStatus;
	uint8 SectorNumber, NumberOfSectors;
	BL_EraseFlag_t EraseFlash_Status = ERASE_FLAG_UNFOUND;

	CmdLen = copy_pu8CmdPacket[0]+1;

	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	SectorNumber = copy_pu8CmdPacket[2];
	NumberOfSectors = copy_pu8CmdPacket[3];

	if(CRCStatus == CRC_VERIFING_PASS){
		voidSendAck(1);

		/*Toggling led while Erasing*/
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

		/*Start Erasing Initialization*/
		EraseStatus = u8Execute_FlashErase(SectorNumber, NumberOfSectors);
		
		/*Flashing Erase Flag*/
		voidSet_EraseFlag();

		/*Verfiy if Erase Flag been written*/
		EraseFlash_Status = VerfiyErase_Flag();

		if(EraseFlash_Status == ERASE_FLAG_FOUND){
			/*Sending the HAL_ErrorStatus*/
			HAL_UART_Transmit(&huart2,&EraseStatus,1, HAL_MAX_DELAY);
		}else{
			voidSendNotAck();
		}

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	}else{
		voidSendNotAck();
	}
}



/**
 * @brief Handles the memory write command from the host, validating CRC, address, and payload, 
 *        and writes data to the specified memory location.
 * 
 * @param copy_pu8CmdPacket Pointer to the command packet received from the host.
 *        The packet structure includes:
 *        - [0]: Command length
 *        - [1]: Command identifier
 *        - [2-5]: Base address (4 bytes)
 *        - [6]: Payload length (1 byte)
 *        - [7-N]: Payload data
 *        - [N+1-N+4]: CRC (4 bytes)
 * 
 */
void BL_voidHandle_MemoryWrite_CMD(uint8 * copy_pu8CmdPacket){
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ;
	uint32 BaseAddress;
	BL_Address_Status_t Address_Status;
	BL_FlashPayload_Status_t FlashWriting_Status;
	uint8 Payload_Length;

	CmdLen = copy_pu8CmdPacket[0]+1;

	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	/*Extract the base - 4 bytes - address given*/
	BaseAddress = *((uint32 *)(&copy_pu8CmdPacket[2]));
	/*Extract the Payload Length given */
	Payload_Length = copy_pu8CmdPacket[6];

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	if(CRCStatus == CRC_VERIFING_PASS){
		voidSendAck(1);

		/*Validate Address give by the user not to exceed the FLASH or SRAM regions*/
		Address_Status = ValidateAddress(BaseAddress);

		if(Address_Status == Address_VALID){
			FlashWriting_Status = u8Execute_MemoryWrite(&copy_pu8CmdPacket[7], BaseAddress, Payload_Length);
		}else{
			FlashWriting_Status = FLASH_PAYLOAD_WRITE_FAILED;
		}

		HAL_UART_Transmit(&huart2,&FlashWriting_Status,1, HAL_MAX_DELAY);

	}else{
		voidSendNotAck();
	}
}



void BL_voidHandle_EnableRWProtect_CMD(uint8 * copy_pu8CmdPacket){
	uint8 CRCStatus, CmdLen;
	uint32 Host_CRC ;
	uint8 Sector_Details;

	BL_Sector_Protection_Mode_t ProtectionMode;

	CmdLen = copy_pu8CmdPacket[0]+1;

	Host_CRC =  *((uint32 *)(copy_pu8CmdPacket + CmdLen - 4));

	CRCStatus = u8VerifyCRC(copy_pu8CmdPacket , ( CmdLen - 4) , Host_CRC );

	/*Extract Sector R/W Protection*/
	Sector_Details = copy_pu8CmdPacket[2];
	/*Extract Sector Protection Mode*/
	ProtectionMode = copy_pu8CmdPacket[3];


	if(CRCStatus == CRC_VERIFING_PASS){

		voidSendAck(1);

		if (ProtectionMode == SECTOR_W_PROTECTION){
			HAL_FLASH_OB_Unlock();
			while (FLASH->SR & FLASH_SR_BSY_Pos);
			/*set SPRMOD to 0 for write protection */
			Clr_BIT(FLASH->OPTCR, FLASH_OPTCR_SPRMOD_Pos);

            /* Apply protection bits (inverted)*/
            FLASH->OPTCR |= (~Sector_Details << FLASH_OPTCR_nWRP_Pos);

		}else if (ProtectionMode == SECTOR_WR_PROTECTION){
			HAL_FLASH_OB_Unlock();
			/* Set SPRMOD to 1 for PCROP protection */
			Set_BIT(FLASH->OPTCR, FLASH_OPTCR_SPRMOD_Pos);

            /* Apply protection bits (inverted)*/
            FLASH->OPTCR |= (~Sector_Details << FLASH_OPTCR_nWRP_Pos);
		}else{

		}
		HAL_FLASH_OB_Launch();
		HAL_FLASH_OB_Lock();
		//HAL_UART_Transmit(&huart2,&RDP_Status,1, HAL_MAX_DELAY);
	}else{
		voidSendNotAck();
	}
}

void BL_voidHandle_MemoryRead_CMD(uint8 * copy_pu8CmdPacket){
}

void BL_voidHandle_ReadSectorStatus_CMD(uint8 * copy_pu8CmdPacket){
}

void BL_voidHandle_OTPRead_CMD(uint8 * copy_pu8CmdPacket){
}

void BL_voidHandle_DisableRWProtect_CMD(uint8 * copy_pu8CmdPacket){

}


/****************************************************************************************************************/
/************************************ Section : Local Functions Implementation **********************************/
/****************************************************************************************************************/

/**
 * @brief Verifies the CRC of a given data array against the host-provided CRC.
 * 
 * @param copy_pu8DataArr Pointer to the data array to validate.
 * @param copy_u8Length Length of the data array.
 * @param copy_u32HostCRC Host-provided CRC value for comparison.
 * @return uint8 Status of CRC verification:
 *         - CRC_VERIFING_PASS if CRC matches.
 *         - CRC_VERIFING_FAILED otherwise.
 */
static uint8 u8VerifyCRC(uint8 * copy_pu8DataArr, uint8 copy_u8Length, uint32 copy_u32HostCRC){
    uint8 Local_u8CRCStatus = CRC_VERIFING_FAILED;
	uint8 Local_u8Iterator;
	uint32_t Local_u32AccCRC, Local_u32Temp;

	for(Local_u8Iterator=0; Local_u8Iterator < copy_u8Length; Local_u8Iterator++){
		Local_u32Temp = copy_pu8DataArr[Local_u8Iterator];
		Local_u32AccCRC= HAL_CRC_Accumulate(&hcrc, &Local_u32Temp , 1);
	}
	/*Reset CRC Calculation Unit*/
	__HAL_CRC_DR_RESET(&hcrc);

	if(Local_u32AccCRC == copy_u32HostCRC)
		Local_u8CRCStatus = CRC_VERIFING_PASS;
	else
		Local_u8CRCStatus = CRC_VERIFING_FAILED;

	return Local_u8CRCStatus;
}

/**
 * @brief Sending Ack when The host request a service from the bootloader
 * 
 * @param copy_u8ReplyLength 
 */
static void voidSendAck(uint8 copy_u8ReplyLength){
	/*The  sending frame is [ACK BYTE , Replay Length of the next freme]*/
	uint8 Local_u8AckBuffer[2]={BL_ACK , copy_u8ReplyLength};

	/*sending the Local_u8AckBuffer array*/
	HAL_UART_Transmit(&huart2, Local_u8AckBuffer ,2 ,HAL_MAX_DELAY);
}

/**
 * @brief Sending Ack when The host request a service from the bootloader
 * 
 * @param copy_u8ReplyLength 
 */
static void voidSendNotAck(void){
	/*The  sending frame is [ACK BYTE , Replay Length of the next freme]*/
	uint8 Local_u8NAck={BL_NOT_ACK};

	/*sending the Local_u8AckBuffer array*/
	HAL_UART_Transmit(&huart2, &Local_u8NAck ,2 ,HAL_MAX_DELAY);
}


/** @brief Validates whether a given address falls within the permissible memory regions.
 * 
 * @param u32Address The 32-bit address to validate.
 * @return BL_Address_Status_t 
 *         - Address_VALID if the address is within the Flash or SRAM regions.
 *         - Address_INVALID otherwise.
 */
static BL_Address_Status_t ValidateAddress(uint32 u32Address){
	BL_Address_Status_t Address_Status;
	/*Address is VALID if it is within : SRAM or FLASH*/

	if ( (u32Address >= STM32F446xx_FLASH_BASE ) && ( u32Address <= STM32F446xx_FLASH_END )){
		Address_Status = Address_VALID;

	}else if ( (u32Address >= STM32F446xx_SRAM1_BASE ) && ( u32Address <= STM32F446xx_SRAM1_END )){
		Address_Status = Address_VALID;
	}else{
		Address_Status = Address_INVALID;
	}
	return Address_Status;
}

//
static HAL_StatusTypeDef u8Execute_FlashErase(uint8 copy_u8SectorNumber, uint8 copy_u8NumberOfSectors){

	HAL_StatusTypeDef ErrorStatus = HAL_OK;
	uint32_t SectorError;
	FLASH_EraseInitTypeDef Flash_Erase;
	uint32 EraseFlag_Value;

	if((copy_u8SectorNumber > 8) & (copy_u8SectorNumber != 0xff)){
		ErrorStatus = HAL_ERROR;
	}else if ((copy_u8SectorNumber > 7) && (copy_u8SectorNumber != 0xff)){
		ErrorStatus = HAL_ERROR;
	}else{
		if(copy_u8SectorNumber == 0xff){
			Flash_Erase.TypeErase=FLASH_TYPEERASE_MASSERASE;
		}else{
			uint8 RemainingSectors = 8 - copy_u8SectorNumber;
			/*if number of sectors is bigger than max, make it equal to the maximum*/
			if(copy_u8NumberOfSectors > RemainingSectors){
				copy_u8NumberOfSectors = RemainingSectors;
			}else{
				/*Nothing*/
			}

			Flash_Erase.TypeErase = FLASH_TYPEERASE_SECTORS;
			Flash_Erase.Sector = copy_u8SectorNumber;
			Flash_Erase.NbSectors = copy_u8NumberOfSectors;
		}

		Flash_Erase.Banks = FLASH_BANK_1;
		Flash_Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		/*Unlock the flash before erasing*/
		HAL_FLASH_Unlock();
		/*Start Erasing*/
		ErrorStatus = HAL_FLASHEx_Erase(&Flash_Erase, &SectorError);

	}
	return ErrorStatus;
}


/**
 * @brief Executes memory write operation either in FLASH or SRAM based on the provided address.
 * 
 * @param Copy_pu8Buffer Pointer to the buffer containing data to be written to memory.
 * @param Copy_u32Address The target memory address to which data will be written.
 * @param Copy_u8Length The length of the data to be written (in bytes).
 * 
 * @return uint8 The status of the memory write operation (HAL_OK if successful) @HAL_StatusTypeDef
 * 
 */
static uint8 u8Execute_MemoryWrite(uint8 * Copy_pu8Buffer, uint32 Copy_u32Address, uint8 Copy_u8Length){
	HAL_StatusTypeDef ErrorStatus = HAL_OK;

	if((Copy_u32Address >= STM32F446xx_FLASH_BASE)&&(Copy_u32Address <= STM32F446xx_FLASH_END)){
		/*Writing in FLASH Case*/
		uint8 iterator;
		/*Unlock the flash before writing*/
		HAL_FLASH_Unlock();

		for(iterator = 0; iterator <Copy_u8Length; iterator ++){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE , (Copy_u32Address + iterator) , (uint64)Copy_pu8Buffer[iterator] );
		}
	}else if((Copy_u32Address >= STM32F446xx_SRAM1_BASE ) && ( Copy_u32Address <= STM32F446xx_SRAM1_END )){
		/*Writing in SRAM Case*/
		uint8 iterator;

		HAL_FLASH_Unlock();

		uint8 * Destination = (uint8 *)Copy_u32Address;

		for(iterator = 0; iterator <Copy_u8Length; iterator ++){
			Destination[iterator] = Copy_pu8Buffer[iterator];
		}
	}

	return ErrorStatus;
}


/**
 * @brief Sets the erase flag by writing a predefined value to the erase flag memory location.
 * 
 */
static void voidSet_EraseFlag(void){
	uint8 EraseFlag[] = {0x74,0x74,0x74,0x74};
	u8Execute_MemoryWrite(EraseFlag, ERASE_FLAG_ADDR, 4);

}


/**
 * @brief Verifies if the erase flag has been set at the specified memory location.
 * 
 * @return BL_EraseFlag_t 
 *         - ERASE_FLAG_FOUND if the flag value matches `ERASE_FLAG_SET`.
 *         - ERASE_FLAG_UNFOUND if the flag value does not match `ERASE_FLAG_SET
 * 
 */
static BL_EraseFlag_t VerfiyErase_Flag(void){
	BL_EraseFlag_t status = ERASE_FLAG_UNFOUND;
	uint32 EraseFlag_Value;

	EraseFlag_Value = *((volatile uint32*)ERASE_FLAG_ADDR);

	if(EraseFlag_Value == ERASE_FLAG_SET){
		status = ERASE_FLAG_FOUND;
	}else{
		status = ERASE_FLAG_UNFOUND;
	}
	return status;
}
