/*
 * OneWire.h
 *
 *  Created on: Nov 17, 2020
 *      Author: arielarias
 */

#ifndef SRC_ONEWIRE_H_
#define SRC_ONEWIRE_H_


/* C++ detection */
#ifdef __cplusplus
extern "C"
{
#endif


#include "stm32f1xx_hal.h"
#include <stdio.h>

/**
 * @defgroup TM_ONEWIRE_Macros
 * @brief    Library defines
 * @{
 */
#define ONEWIRE_DELAY(x)    micros(x)
/* OneWire commands */
#define ONEWIRE_CMD_RSCRATCHPAD         0xBE
#define ONEWIRE_CMD_WSCRATCHPAD         0x4E
#define ONEWIRE_CMD_CPYSCRATCHPAD       0x48
#define ONEWIRE_CMD_RECEEPROM           0xB8
#define ONEWIRE_CMD_RPWRSUPPLY          0xB4
#define ONEWIRE_CMD_SEARCHROM           0xF0
#define ONEWIRE_CMD_READROM             0x33
#define ONEWIRE_CMD_MATCHROM            0x55
#define ONEWIRE_CMD_SKIPROM             0xCC

/**
 * @}
 */

/**
 * @defgroup TM_ONEWIRE_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  OneWire working struct
 * @note   Except ROM_NO member, everything is fully private and should not be touched by user
 */
typedef struct
{
    GPIO_TypeDef *GPIOx; /*!< GPIOx port to be used for I/O functions */
    uint16_t GPIO_Pin; /*!< GPIO Pin to be used for I/O functions */
    uint8_t LastDiscrepancy; /*!< Search private */
    uint8_t LastFamilyDiscrepancy; /*!< Search private */
    uint8_t LastDeviceFlag; /*!< Search private */
    uint8_t ROM_NO[8]; /*!< 8-bytes address of last search device */
} OneWire_t;

/**
 * @}
 */

/**
 * @defgroup TM_ONEWIRE_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes OneWire bus
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t empty working onewire structure
 * @param  *Pointer to GPIO port used for onewire channel
 * @param  GPIO_Pin: GPIO Pin on specific GPIOx to be used for onewire channel
 * @retval None
 */

void OneWire_Low( OneWire_t * gp );

void OneWire_High( OneWire_t * gp );

void OneWire_Input( OneWire_t * gp );

void OneWire_Output( OneWire_t * gp );

void micros( uint16_t time );

void OneWire_Init( OneWire_t * OneWireStruct, GPIO_TypeDef * GPIOx,
        uint16_t GPIO_Pin, TIM_HandleTypeDef _timer );

/**
 * @brief  Resets OneWire bus
 *
 * @note   Sends reset command for OneWire
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire structure
 * @retval None
 */
uint8_t OneWire_Reset( OneWire_t * OneWireStruct );

/**
 * @brief  Reads byte from one wire bus
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire structure
 * @retval Byte from read operation
 */
uint8_t OneWire_ReadByte( OneWire_t * OneWireStruct );

/**
 * @brief  Writes byte to bus
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire structure
 * @param  byte: 8-bit value to write over OneWire protocol
 * @retval None
 */
void OneWire_WriteByte( OneWire_t * OneWireStruct, uint8_t byte );

/**
 * @brief  Writes single bit to onewire bus
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire structure
 * @param  bit: Bit value to send, 1 or 0
 * @retval None
 */
void OneWire_WriteBit( OneWire_t * OneWireStruct, uint8_t bit );

/**
 * @brief  Reads single bit from one wire bus
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire structure
 * @retval Bit value:
 *            - 0: Bit is low (zero)
 *            - > 0: Bit is high (one)
 */
uint8_t OneWire_ReadBit( OneWire_t * OneWireStruct );

/**
 * @brief  Searches for OneWire devices on specific Onewire port
 * @note   Not meant for public use. Use @ref TM_OneWire_First and @ref TM_OneWire_Next for this.
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire structure where to search
 * @param  Device status:
 *            - 0: No devices detected
 *            - > 0: Device detected
 */
uint8_t OneWire_Search( OneWire_t * OneWireStruct, uint8_t command );

/**
 * @brief  Resets search states
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire where to reset search values
 * @retval None
 */
void OneWire_ResetSearch( OneWire_t * OneWireStruct );

/**
 * @brief  Starts search, reset states first
 * @note   When you want to search for ALL devices on one onewire port, you should first use this function.
 \code
 //...Initialization before
 status = TM_OneWire_First(&OneWireStruct);
 while (status) {
 //Save ROM number from device
 TM_OneWire_GetFullROM(ROM_Array_Pointer);
 //Check for new device
 status = TM_OneWire_Next(&OneWireStruct);
 }
 \endcode
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire where to reset search values
 * @param  Device status:
 *            - 0: No devices detected
 *            - > 0: Device detected
 */
uint8_t OneWire_First( OneWire_t * OneWireStruct );

/**
 * @brief  Reads next device
 * @note   Use @ref TM_OneWire_First to start searching
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire
 * @param  Device status:
 *            - 0: No devices detected any more
 *            - > 0: New device detected
 */
uint8_t OneWire_Next( OneWire_t * OneWireStruct );

/**
 * @brief  Gets ROM number from device from search
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire
 * @param  index: Because each device has 8-bytes long ROm address, you have to call this 8 times, to get ROM bytes from 0 to 7
 * @reetval ROM byte for index (0 to 7) at current found device
 */
uint8_t OneWire_GetROM( OneWire_t * OneWireStruct, uint8_t index );

/**
 * @brief  Gets all 8 bytes ROM value from device from search
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire
 * @param  *firstIndex: Pointer to first location for first byte, other bytes are automatically incremented
 * @retval None
 */
void OneWire_GetFullROM( OneWire_t * OneWireStruct, uint8_t * firstIndex );

/**
 * @brief  Selects specific slave on bus
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire
 * @param  *addr: Pointer to first location of 8-bytes long ROM address
 * @retval None
 */
void OneWire_Select( OneWire_t * OneWireStruct, uint8_t * addr );

/**
 * @brief  Selects specific slave on bus with pointer address
 * @param  *OneWireStruct: Pointer to @ref TM_OneWire_t working onewire
 * @param  *ROM: Pointer to first byte of ROM address
 * @retval None
 */
void OneWire_SelectWithPointer( OneWire_t * OneWireStruct, uint8_t * ROM );

/**
 * @brief  Calculates 8-bit CRC for 1-wire devices
 * @param  *addr: Pointer to 8-bit array of data to calculate CRC
 * @param  len: Number of bytes to check
 *
 * @retval Calculated CRC from input data
 */
uint8_t OneWire_CRC8( uint8_t * addr, uint8_t len );


/* C++ detection */
#ifdef __cplusplus
}
#endif




#endif /* SRC_ONEWIRE_H_ */
