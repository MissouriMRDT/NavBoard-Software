#ifndef _RoveWire
#define _RoveWire

#include "Wire.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

uint32_t I2CReceive(uint8_t SlaveAddr, uint8_t reg);
void I2CSend(uint8_t slave_addr, uint8_t reg, uint8_t data);

#endif