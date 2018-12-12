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

#include "LSMI2C.h"

//list of all registers binary addresses;
#include "IMUreg.h"
#include "Quaternion.h"

class LSM90S1
{
  private:
	float gyro[3];
    float accel[3];
    float mag[3];
	
	float gyroCal[3];
    float accelCal[3];
    float magCal[3];
	int16_t temperature;
	
	
	
    float gOffset[3];
    float aOffset[3];
    float mOffset[3];
	
	float gBias[3]={308.2666320800781250, 213.8016662597656250, -627.1350708007812500};
    float aBias[3]={-399.9445800781250000, -534.8186645507812500, 16720.2167968750000000};
    float mBias[3]={2013.5000000000000000, 1240.0000000000000000, 2292.0000000000000000};
	
	// Set initial input parameters
    enum Ascale {  // set of allowable accel full scale settings
      AFS_2G = 0,
      AFS_16G,
      AFS_4G,
      AFS_8G
    };
    
    enum Aodr {  // set of allowable gyro sample rates
      AODR_PowerDown = 0,
      AODR_10Hz,
      AODR_50Hz,
      AODR_119Hz,
      AODR_238Hz,
      AODR_476Hz,
      AODR_952Hz
    };
    
    enum Abw {  // set of allowable accewl bandwidths
       ABW_408Hz = 0,
       ABW_211Hz,
       ABW_105Hz,
       ABW_50Hz
    };
    
    enum Gscale {  // set of allowable gyro full scale settings
      GFS_245DPS = 0,
      GFS_500DPS,
      GFS_NoOp,
      GFS_2000DPS
    };
    
    enum Godr {  // set of allowable gyro sample rates
      GODR_PowerDown = 0,
      GODR_14_9Hz,
      GODR_59_5Hz,
      GODR_119Hz,
      GODR_238Hz,
      GODR_476Hz,
      GODR_952Hz
    };
    
    enum Gbw {   // set of allowable gyro data bandwidths
      GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
      GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
      GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
      GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
    };
    
    enum Mscale {  // set of allowable mag full scale settings
      MFS_4G = 0,
      MFS_8G,
      MFS_12G,
      MFS_16G
    };
    
    enum Mmode {
      MMode_LowPower = 0, 
      MMode_MedPerformance,
      MMode_HighPerformance,
      MMode_UltraHighPerformance
    };
    
    enum Modr {  // set of allowable mag sample rates
      MODR_0_625Hz = 0,
      MODR_1_25Hz,
      MODR_2_5Hz,
      MODR_5Hz,
      MODR_10Hz,
      MODR_20Hz,
      MODR_80Hz
    };
    
    #define ADC_256  0x00 // define pressure and temperature conversion rates
    #define ADC_512  0x02
    #define ADC_1024 0x04
    #define ADC_2048 0x06
    #define ADC_4096 0x08
    #define ADC_D1   0x40
    #define ADC_D2   0x50
    
    // Specify sensor full scale
    uint8_t OSR = ADC_4096;      // set pressure amd temperature oversample rate
    uint8_t Gscale = GFS_245DPS; // gyro full scale
    uint8_t Godr = GODR_238Hz;   // gyro data sample rate
    uint8_t Gbw = GBW_med;       // gyro data bandwidth
    uint8_t Ascale = AFS_2G;     // accel full scale
    uint8_t Aodr = AODR_238Hz;   // accel data sample rate
    uint8_t Abw = ABW_50Hz;      // accel data bandwidth
    uint8_t Mscale = MFS_4G;     // mag full scale
    uint8_t Modr = MODR_10Hz;    // mag data sample rate
    uint8_t Mmode = MMode_HighPerformance;  // magnetometer operation mode
    float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
	
	void initLSM9DS1();
  
    
  public:
    Quaternion quaternion;
	
    void begin();
    void readGyro(float gyro[3]);
    void readAccel(float accel[3]);
    void readMag(float mag[3]);
    void readTemp(int16_t &temperature);
	void read();
	void updateMadgwick();
	
	float getPitch();
    float getRoll();
    float getHeading();
    float getTrueHeading();
	
	void calibrateGyro(int ms);   //ToDo: write calibrate funtions
	void calibrateAccel(int ms);
	void calibrateMag(int ms);
		
	void printRaw();
	void printCal();
	
	void getAres();
	void getMres();
	void getGres();
};

#endif