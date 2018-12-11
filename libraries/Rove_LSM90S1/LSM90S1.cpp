#include "LSM90S1.h"

#include "Wire.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "Energia.h"

//list of all registers binary addresses;
#include "IMUreg.h"

#include "RoveWire.h"

void LSM90S1::begin()
{
  I2CSend(Address_AG, CTRL_REG4,0B00111000);  //enable gyro axis
  I2CSend(Address_AG, CTRL_REG5_XL,0B00111000); //enable accelerometer
  I2CSend(Address_AG, CTRL_REG1_G,0B01100000); //gyro/accel odr and bw
  I2CSend(Address_M, CTRL_REG3_M,0B00000000); //enable mag continuous
}

void LSM90S1::readGyro(float gyro[3])
{
  byte X_L = I2CReceive(Address_AG, OUT_X_L_G);//gyroscope pitch
  byte X_H = I2CReceive(Address_AG, OUT_X_H_G);
  byte Y_L = I2CReceive(Address_AG, OUT_Y_L_G);
  byte Y_H = I2CReceive(Address_AG, OUT_Y_H_G);
  byte Z_L = I2CReceive(Address_AG, OUT_Z_L_G);
  byte Z_H = I2CReceive(Address_AG, OUT_Z_H_G);

  int16_t X_AXIs = X_H <<8 | X_L;
  int16_t Y_AXIs = Y_H <<8 | Y_L;
  int16_t Z_AXIs = Z_H <<8 | Z_L;


  float real_X_Axis =0.00875*(X_AXIs-320);
  float real_Y_Axis =0.00875*(Y_AXIs-17);
  float real_Z_Axis =0.00875*(Z_AXIs+190);
  
  gyro[0] = real_X_Axis;
  gyro[1] = real_Y_Axis;
  gyro[2] = real_Z_Axis;
}

void LSM90S1::readAccel(float accel[3])
{
  byte X_L_A = I2CReceive(Address_AG, OUT_X_L_XL);//Output acceleration in x-axis as a 16-bit word in two's complement
  byte X_H_A = I2CReceive(Address_AG, OUT_X_H_XL);
  byte Y_L_A = I2CReceive(Address_AG, OUT_Y_L_XL);
  byte Y_H_A = I2CReceive(Address_AG, OUT_Y_H_XL);
  byte Z_L_A = I2CReceive(Address_AG, OUT_Z_L_XL);
  byte Z_H_A = I2CReceive(Address_AG, OUT_Z_H_XL);

  
  int16_t X_AXIS_A = X_H_A <<8 | X_L_A;
  int16_t Y_AXIS_A = Y_H_A <<8 | Y_L_A;
  int16_t Z_AXIS_A = Z_H_A <<8 | Z_L_A;
  
  float real_X_AXIS_A = X_AXIS_A*0.000061;
  float real_Y_AXIS_A = Y_AXIS_A*0.000061;
  float real_Z_AXIS_A = Z_AXIS_A*0.000061;
  

  accel[0] = real_X_AXIS_A;
  accel[1] = real_Y_AXIS_A;
  accel[2] = real_Z_AXIS_A; 
}

void LSM90S1::readMag(float mag[3])
{
  byte X_L_M = I2CReceive(Address_M, OUT_X_L_M);//Magnetometer data expressed as two's complement
  byte X_H_M = I2CReceive(Address_M, OUT_X_H_M);
  byte Y_L_M = I2CReceive(Address_M, OUT_Y_L_M);
  byte Y_H_M = I2CReceive(Address_M, OUT_Y_H_M);
  byte Z_L_M = I2CReceive(Address_M, OUT_Z_L_M);
  byte Z_H_M = I2CReceive(Address_M, OUT_Z_H_M);
  
  int16_t X_AXIS_M = X_H_M <<8 | X_L_M;
  int16_t Y_AXIS_M = Y_H_M <<8 | Y_L_M;
  int16_t Z_AXIS_M = Z_H_M <<8 | Z_L_M;
  
  float real_X_Axis_M = X_AXIS_M*0.00014;
  float real_Y_Axis_M = Y_AXIS_M*0.00014;
  float real_Z_Axis_M = Z_AXIS_M*0.00014;
  
  float MAG_DATA[3];
  mag[0]= real_X_Axis_M;
  mag[1] = real_Y_Axis_M;
  mag[2] = real_Z_Axis_M;
}

void LSM90S1::readTemp(int16_t &temperature)
{
  byte Temp_L = I2CReceive(Address_AG, OUT_TEMP_L);
  byte Temp_H = I2CReceive(Address_AG, OUT_TEMP_H);

  int16_t Temp = Temp_H <<8 | Temp_L;
  temperature = (Temp/16.0)+25;
}

LSM90S1Data LSM90S1::read()
{
  LSM90S1Data IMUData;
  
  readGyro (IMUData.gyro);
  readMag  (IMUData.accel);
  readAccel(IMUData.mag);
  //readTemp (IMUData.temperature);
  
  return IMUData;
}

void LSM90S1::print(LSM90S1Data Data)
{
  Serial.print("\n---IMU Data---\n             ----X----           ----Y----          ----Z----");
  Serial.print("\nGyro:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(Data.gyro[i], 16);
  }
  
  Serial.print("\nAcce:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(Data.accel[i], 16);
  }
  
  Serial.print("\nMagn:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(Data.mag[i], 16);
  }
}