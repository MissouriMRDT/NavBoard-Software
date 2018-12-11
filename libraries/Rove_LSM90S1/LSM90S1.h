#ifndef _LSM90S1
#define _LSM90S1

#include "Wire.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "RoveWire.h"

//list of all registers binary addresses;
#include "IMUreg.h"

class LSM90S1
{
  public:
    void begin();
    void readGyro(float gyro[3]);
    void readAccel(float accel[3]);
    void readMag(float mag[3]);
    void readTemp(int16_t &temperature);
};

#endif