#include "RoveWire.h"

#include "Wire.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

uint32_t I2CReceive(uint8_t SlaveAddr, uint8_t reg)
{
    uint32_t i2cBase = I2C0_BASE;
  //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, false);

    //specify register to be read on the slave device
    I2CMasterDataPut(i2cBase, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to start transaction
    while(!I2CMasterBusy(i2cBase));

  //wait for MCU to finish transaction
    while(I2CMasterBusy(i2cBase));

    //specify that we are going to read from slave device with 3rd argument (read/~write)= true
    I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, true);

    //send control byte and read from the register we specified earlier
    I2CMasterControl(i2cBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

  //wait for MCU to start transaction
    while(!I2CMasterBusy(i2cBase));

    //wait for MCU to finish transaction
    while(I2CMasterBusy(i2cBase));

    //return data pulled from the specified register, returns uint32_t
    return I2CMasterDataGet(i2cBase);

}

void I2CSend(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
    uint32_t i2cBase = I2C0_BASE;

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(i2cBase, slave_addr, false);

    //put data to be sent into FIFO
    I2CMasterDataPut(i2cBase, reg);

        //Initiate send of data from the MCU
        I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_START);

  //wait for MCU to start transaction
    while(!I2CMasterBusy(i2cBase));

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(i2cBase));

        I2CMasterDataPut(i2cBase, data);

        I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_FINISH);

  //wait for MCU to start transaction
    while(!I2CMasterBusy(i2cBase));

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(i2cBase));
}