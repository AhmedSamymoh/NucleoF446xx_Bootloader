# NucleoF446xx_Bootloader
A bootloader for STM32F4 stored in Flash memory enabling firmware update, with implemented commands for memory operations, address jumps, and communication via CRC-validated command packets.


### For STM32F4:
- You can not store bootloader at the Rom(system memory) where ST's Bootloader stored
- so we can use Flash memory itself to store our bootloader

| Sector Number | Start Address | Size   | Description           |
| ------------- | ------------- | ------ | --------------------- |
| Sector 0      | 0x08000000    | 16 KB  | Bootloader or app.    |
| Sector 1      | 0x08004000    | 16 KB  | Bootloader or app.    |
| Sector 2      | 0x08008000    | 16 KB  | Application main code |
| Sector 3      | 0x0800C000    | 16 KB  | Application main code |
| Sector 4      | 0x08010000    | 64 KB  | Application main code |
| Sector 5      | 0x08020000    | 128 KB | Application main code |
| Sector 6      | 0x08040000    | 128 KB | Application main code |
| Sector 7      | 0x08060000    | 128 KB | Application main code |

---------
- When you reset the `uC`, it'll make its initiations, clock ,communication, then it'll wait for data from host on communication
- When the host sends `command packet` , the bootloader will receive it and decode it, check for data validation `CRC` of the received Packet 
- if data was good, it'll send `ACK`, if it fails, it will send `NACK`
- if `ACK` is sent, it'll be followed by `the length to follow`

![pic](./utils/Pasted%20image%2020241201130309.png)

-----------------
# Implemented Commands:
- BL_GetVersion : Retrieves the bootloader version
- BL_GetHelp :Provides a list of supported bootloader commands
- BL_GetChipID : Returns Chip ID
- BL_GetRDPStatus : Reads the current read protection level
- BL_GoToAddress : jumps to a specific address in memory
- BL_EnableRWProtect: Enable write/read protection for a specific sector
- BL_GoToAddress: to jump to a specified address in memory.
- BL_FlashErase: for erasing specific sectors in Flash memory.
- BL_MemoryWrite: to write data into Flash memory.
- BL_u8Execute_FlashErase: Handles erasing specific flash memory sectors.
- BL_u8Execute_MemoryWrite: Handles writing data to a specified memory location.

- -----
## Examples of Host-Bootloader Frame 
### BL_GET_VER:
- used to det Bootloader version
- command code `0x51`

![pic](./utils/Pasted%20image%2020241201131245.png)

### BL_GoToAddress:
- used to jump to a specific address in memory
- you should initialize the main stack pointer before jumping to that address
- command code `0x55`

![pic](./utils/Pasted%20image%2020241201132208.png)

### BL_FlashErase:
- for erasing specific sectors in Flash memory.
- Sending sector number (2,3,4,5,6,7) and Number of sectors (1-6)
- command code `0x56`

![pic](./utils/Pasted%20image%2020241201133546.png)

### BL_MemoryWrite:
- Write data or Application code into Flash memory.
- Sending Basememory address and payload length and the Payload
- command code `0x56`

![pic](./utils/Pasted%20image%2020241201133913.png)

------
# Flash Memory Utilization

![pic](./utils/Pasted%20image%2020241201133342.png)


-----
## Tips:

- **Read MSP Value:** Load the initial stack pointer from `0x08008000` into `Local_u32MSPVal`.
- **Prepare Pointer:** Declare `App_ResetHandler` for the reset handler address.
- **Set MSP Register:** Use inline assembly to write `Local_u32MSPVal` into MSP.
- **Retrieve Reset Handler:** Get the reset handler address from `0x08008004`.
- **Assign and Jump:** Assign address to `App_ResetHandler` and call it to transfer control.

```c
void Bootloader_JumpToApp(void){
  
/* Configure MSP of user app by reading value from base addres of sector2 */
uint32_t Local_u32MSPVal = *((volatile uint32_t*)(0x08008000));

uint32_t ResetHandlerAddress;

/*Pointer to function to hold the address of the reset handler of the user app*/
void (*App_ResetHandler)();
  
/* Write the user MSP Value into MSP register
* MSR -> Move Special Register
* MSP -> Main stack pointer register
* %0 receive variable:output:input
* "r" for register
* */
__asm volatile("MSR MSP, %0"::"r"(Local_u32MSPVal));
  
/*Get reset handler address of user app*/
ResetHandlerAddress = *((volatile uint32_t *)( 0x08008000UL + 4 ));

App_ResetHandler = (void *)ResetHandlerAddress;

/*Jump to the user app reset handler*/
App_ResetHandler();
}
```
