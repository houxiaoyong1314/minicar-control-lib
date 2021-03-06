#ifndef HAL_I2C_H
#define HAL_I2C_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <ioCC2541.h>
#include "Common.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_I2C_SLAVE_ADDR_DEF           0x41

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
typedef enum
{
  i2cClock_123KHZ = 0x00,
  i2cClock_144KHZ = 0x01,
  i2cClock_165KHZ = 0x02,
  i2cClock_197KHZ = 0x03,
  i2cClock_33KHZ  = 0x80,
  i2cClock_267KHZ = 0x81,
  i2cClock_533KHZ = 0x82
} i2cClock_t;


/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */
void    HalI2CInit(uint8 address, i2cClock_t clockRate);
uint8   HalI2CRead(uint8 addr,uint8 len, uint8 *pBuf);
uint8   HalI2CWrite(uint8 len, uint8 *pBuf);
void    HalI2CDisable(void);

void WriteData(uint8 Addr,uint8 Dat);
void ReadData(uint8 Addr,uint8 *Pbuf,uint8 Num);
#endif
/**************************************************************************************************
 */
