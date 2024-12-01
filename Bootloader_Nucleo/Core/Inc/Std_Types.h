/***********************************************************************************************
 * @file           : STD_TYPES.h
 * @brief          : Standard Types header file.
 * @version        : 1.0.0
 * @module         : Standard Types
 * @platform       : stm32f446re
 * @autosar_version: R23-11
 * @sw_version     : 1.0.0
 * @created on     : Aug 20, 2024
 * @author         : AhmedSamy
 ***********************************************************************************************/

#ifndef STD_TYPES_H
#define STD_TYPES_H



/************************************ Section : Includes ************************************/

#include "Platform_Types.h"


/************************************ Section: Data Type Declarations ************************************/


/*
* Std_ReturnType
* [SWS_Std_00005]
*/
typedef enum{
    E_OK = 0x00,
    E_NOT_OK = 0x01
} Std_ReturnType;

/************************************ Section: Macro Declarations ************************************/
/*
* (SRS_BSW_00004)
* Std_VersionInfoType :
* All Basic SW Modules shall perform a pre-processor
* check of the versions of all imported include files
*/
#define STD_TYPES_SW_vendor_ID        1u
#define STD_TYPES_SW_moduleID         1u
#define STD_TYPES_SW_major_version    1u
#define STD_TYPES_SW_minor_version    1u
#define STD_TYPES_SW_patch_version    1u

/*
* [SWS_Std_00007]
*/
#define STD_HIGH            0x01u /* Physical state 5V or 3.3V */
#define STD_LOW             0x00u /* Physical state 0V */

/*
* [SWS_Std_00013]
*/
#define STD_ACTIVE          0x01u /* Logical state active */
#define STD_IDLE            0x00u /* Logical state idle */

/*
* [SWS_Std_00031]
*/
#define NULL                ((void *)0)

/*
* [SWS_Std_00010]
*/
#define STD_ON              0x01u
#define STD_OFF             0x00u

#define BIT_MASK            0x01


/************************************ Section : Macro Functions Declarations ************************************/

/**
 * @brief Bit Math Macros for setting, clearing, toggling and reading a bit.
 */
#define HWREG8(_address)                ( *((volatile uint8*)(_address)) )

#define Set_BIT(REG , BIT_POSN)         (REG |=  (BIT_MASK << BIT_POSN))
#define Clr_BIT(REG , BIT_POSN)         (REG &= ~(BIT_MASK << BIT_POSN))
#define Tog_BIT(REG , BIT_POSN)         (REG ^=  (BIT_MASK << BIT_POSN))
#define Read_BIT(REG , BIT_POSN)        ((REG >> BIT_POSN ) & BIT_MASK )




/* Section: Function Declarations */







#endif /* STD_TYPES_H_ */
