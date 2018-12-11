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
#include "Quaternion.h"

#include "LSMI2C.h"

void LSM90S1::begin()//ToDo: set resolutions
{
  // Read the WHO_AM_I registers, this is a good test of communication 
  Serial.println("LSM9DS1 is online...");
   
	// get sensor resolutions, only need to do this once
  getAres();
  getGres();
  getMres();
  Serial.print("accel sensitivity is "); Serial.print(1./(1000.*aRes)); Serial.println(" LSB/mg");
  Serial.print("gyro sensitivity is "); Serial.print(1./(1000.*gRes)); Serial.println(" LSB/mdps");
  Serial.print("mag sensitivity is "); Serial.print(1./(1000.*mRes)); Serial.println(" LSB/mGauss");

  I2CSend(Address_AG, CTRL_REG4,0B00111000);  //enable gyro axis
  I2CSend(Address_AG, CTRL_REG5_XL,0B00111000); //enable accelerometer
  I2CSend(Address_AG, CTRL_REG1_G,0B01100000); //gyro/accel odr and bw
  I2CSend(Address_M, CTRL_REG3_M,0B00000000); //enable mag continuous
  
  // enable the 3-axes of the gyroscope
  I2CSend(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  I2CSend(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer 
  I2CSend(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  I2CSend(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  I2CSend(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
  // configure the magnetometer-enable temperature compensation of mag data
  I2CSend(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  I2CSend(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
  I2CSend(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  I2CSend(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  I2CSend(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
   
  Serial.println("Finished Init");

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

void LSM90S1::read()
{  
  readGyro (gyro);
  readMag  (accel);
  readAccel(mag);
  
  for(int i = 0; i<3; i++)
  {
    gyroCal [i] = gyro [i]*gRes-gBias[i];
    accelCal[i] = accel[i]*aRes-aBias[i];
    magCal  [i] = mag  [i]*mRes;//-mBias[i];
  }
  //readTemp (IMUData.temperature);
  updateMadgwick();

}

void LSM90S1::updateMadgwick()
{
  quaternion.updateMadgwick(accelCal[0], accelCal[1], accelCal[2], radians(gyroCal[0]), radians(gyroCal[1]), radians(gyroCal[2]), -magCal[0], magCal[1], magCal[2]);
}

float LSM90S1::getHeading()
{
    return quaternion.heading;
}

float LSM90S1::getPitch()
{
    return quaternion.pitch;
}

float LSM90S1::getRoll()
{
    return quaternion.roll;
}

float LSM90S1::getTrueHeading()
{
    return quaternion.trueHeading;
}

void LSM90S1::calibrateGyro(int ms)
{
  Serial.println("\n---Calibrating Gyro");
  delay(100);
  
  float gyro_bias[3] = {0, 0, 0};
  uint16_t samples=0;
  
  int startTime = millis();
  while((millis()-startTime)<ms)
  {
    float gyro_temp[3] = {0, 0, 0};
    readGyro(gyro_temp);

    gyro_bias[0] += gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += gyro_temp[1]; 
    gyro_bias[2] += gyro_temp[2]; 
	//Serial.println(gyro_temp[0]);
	samples++;
	Serial.print((millis()-startTime)*100/ms);
	Serial.println("%");
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  gBias[0] = gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  gBias[1] = gyro_bias[1]*gRes;
  gBias[2] = gyro_bias[2]*gRes;
  
  Serial.print("\ngBias:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(gyro_bias[i], 16);
  }
}

void LSM90S1::calibrateAccel(int ms)
{
  Serial.println("\n---Calibrating Accel");
  delay(100);
  
  float accel_bias[3] = {0, 0, 0};
  uint16_t samples=0;
  
  int startTime = millis();
  while((millis()-startTime)<ms)
  {
    float accel_temp[3] = {0, 0, 0};
    readAccel(accel_temp);

    accel_bias[0] += accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += accel_temp[1]; 
    accel_bias[2] += accel_temp[2]; 
	//Serial.println(accel_temp[0]);
	samples++;
	Serial.print((millis()-startTime)*100/ms);
	Serial.println("%");
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  aBias[0] = accel_bias[0]*aRes;  // Properly scale the data to get deg/s
  aBias[1] = accel_bias[1]*aRes;
  aBias[2] = accel_bias[2]*aRes;
  
  Serial.print("\naBias:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(accel_bias[i], 16);
  }
}

void LSM90S1::calibrateMag(int ms)
{
  Serial.println("\n---Calibrating Mag");
  delay(100);
  
  float mag_min[3] = {0, 0, 0};
  float mag_max[3] = {0, 0, 0};
  float mag_bias[3] = {0, 0, 0};
  uint16_t samples=0;
  
  int startTime = millis();
  while((millis()-startTime)<ms)
  {
    float mag_temp[3] = {0, 0, 0};
    readMag(mag_temp);
	
	for(int i = 0; i<3; i++)
	{
      if(mag_min[i] >= mag_temp[i]) mag_min[i] = mag_temp[i];
      if(mag_max[i] <= mag_temp[i]) mag_max[i] = mag_temp[i]; 
	}
	//Serial.println(gyro_temp[0]);
	samples++;
	Serial.print((millis()-startTime)*100/ms);
	Serial.println("%");
  }  
  
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
  
  mBias[0] = mag_bias[0]*mRes;  // Properly scale the data to get deg/s
  mBias[1] = mag_bias[1]*mRes;
  mBias[2] = mag_bias[2]*mRes;
  
  Serial.print("\nmBias:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(mag_bias[i], 16);
  }
}
void LSM90S1::printRaw()
{
  Serial.print("\n---Raw IMU Data---\n             ----X----           ----Y----          ----Z----");
  Serial.print("\nGyro:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(gyro[i], 16);
  }
  
  Serial.print("\nAcce:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(accel[i], 16);
  }
  
  Serial.print("\nMagn:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(mag[i], 16);
  }
  Serial.println("");
}

void LSM90S1::printCal()
{
  Serial.print("\n---Cal IMU Data---\n             ----X----           ----Y----          ----Z----");
  Serial.print("\nGyro:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(gyroCal[i], 16);
  }
  
  Serial.print("\nAcce:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(accelCal[i], 16);
  }
  
  Serial.print("\nMagn:");
  for(int i=0; i<3; i++)
  {
	Serial.print(", ");
	Serial.print(magCal[i], 16);
  }
  Serial.println("");
}

void LSM90S1::getMres() 
{
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
    case MFS_4G:
          mRes = 4.0/32768.0;
          break;
    case MFS_8G:
          mRes = 8.0/32768.0;
          break;
    case MFS_12G:
          mRes = 12.0/32768.0;
          break;
    case MFS_16G:
          mRes = 16.0/32768.0;
          break;
  }
}

void LSM90S1::getGres() 
{
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
    case GFS_245DPS:
          gRes = 245.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void LSM90S1::getAres()
{
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
  }
}
