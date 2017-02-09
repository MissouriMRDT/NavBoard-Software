#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "RoveEthernet.h"
#include "RoveComm.h"

#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>
#include "Wire.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include <Wire.h> //add arduino I2C library;
//list of all registers binary addresses;
byte ACT_THS              = 0B00000100;
byte ACT_DUR              = 0B00000101;
byte INT_GEN_CFG_XL       = 0B00000110;
byte INT_GEN_THS_X_XL     = 0B00000111;
byte INT_GEN_THS_Y_XL     = 0B00001000;
byte INT_GEN_THS_Z_XL     = 0B00001001;
byte INT_GEN_DUR_XL       = 0B00001010;
byte REFERENCE_G          = 0B00001011;
byte INT1_CTRL            = 0B00001100;
byte INT2_CTRL            = 0B00001101;

byte WHO_AM_I             = 0B00001111;
byte CTRL_REG1_G          = 0B00010000;
byte CTRL_REG2_G          = 0B00010001;
byte CTRL_REG3_G          = 0B00010010;
byte ORIENT_CFG_G         = 0B00010011;
byte INT_GEN_SRC_G        = 0B00010100;
byte OUT_TEMP_L           = 0B00010101;
byte OUT_TEMP_H           = 0B00010110;
byte STATUS_REG           = 0B00010111;
byte OUT_X_L_G            = 0B00011000;
byte OUT_X_H_G            = 0B00011001;
byte OUT_Y_L_G            = 0B00011010;
byte OUT_Y_H_G            = 0B00011011;
byte OUT_Z_L_G            = 0B00011100;
byte OUT_Z_H_G            = 0B00011101;
byte CTRL_REG4            = 0B00011110;
byte CTRL_REG5_XL         = 0B00011111;
byte CTRL_REG6_XL         = 0B00100000;
byte CTRL_REG7_XL         = 0B00100001;
byte CTRL_REG8            = 0B00100010;
byte CTRL_REG9            = 0B00100011;
byte CTRL_REG10           = 0B00100100;

byte INT_GEN_SRC_XL       = 0B00100110;
byte STATUS_REG0          = 0B00100111;
byte OUT_X_L_XL           = 0B00101000;
byte OUT_X_H_XL           = 0B00101001;
byte OUT_Y_L_XL           = 0B00101010;
byte OUT_Y_H_XL           = 0B00101011;
byte OUT_Z_L_XL           = 0B00101100;
byte OUT_Z_H_XL           = 0B00101101;
byte FIFO_CTRL            = 0B00101110;
byte FIFO_SRC             = 0B00101111;
byte INT_GEN_CFG_G        = 0B00110000;
byte INT_GEN_THS_XH_G     = 0B00110001;
byte INT_GEN_THS_XL_G     = 0B00110010;
byte INT_GEN_THS_YH_G     = 0B00110011;
byte INT_GEN_THS_YL_G     = 0B00110100;
byte INT_GEN_THS_ZH_G     = 0B00110101;
byte INT_GEN_THS_ZL_G     = 0B00110110;
byte INT_GEN_DUR_G        = 0B00110111;


byte OFFSET_X_REG_L_M     = 0B00000101;
byte OFFSET_X_REG_H_M     = 0B00000110;
byte OFFSET_Y_REG_L_M     = 0B00000111;
byte OFFSET_Y_REG_H_M     = 0B00001000;
byte OFFSET_Z_REG_L_M     = 0B00001001;
byte OFFSET_Z_REG_H_M     = 0B00001010;

byte WHO_AM_I_M           = 0B00001111;

byte CTRL_REG1_M          = 0B00100000;
byte CTRL_REG2_M          = 0B00100001;
byte CTRL_REG3_M          = 0B00100010;
byte CTRL_REG4_M          = 0B00100011;
byte CTRL_REG5_M          = 0B00100100;

byte STATUS_REG_M         = 0B00100111;
byte OUT_X_L_M            = 0B00101000;
byte OUT_X_H_M            = 0B00101001;
byte OUT_Y_L_M            = 0B00101010;
byte OUT_Y_H_M            = 0B00101011;
byte OUT_Z_L_M            = 0B00101100;
byte OUT_Z_H_M            = 0B00101101;

byte INT_CFG_M            = 0B00110000;
byte INT_SRC_M            = 0B00110001;
byte INT_THS_L_M          = 0B00110010;
byte INT_THS_H_M          = 0B00110011;

byte Read    = 0B00000001;
byte Write   = 0B00000000;
byte Address_AG =   0B01101011;  //address of accelerometer/gyro with SDO_AG connected to Vdd
byte Address_M   =   0B00011110;  //address of magnetometer with SDO_M connected to Vdd

const uint16_t GPS_FIX_QUALITY_DATA_ID = 301;
const uint16_t GPS_LAT_LON_DATA_ID = 302;
const uint16_t GPS_SPEED_DATA_ID = 303;
const uint16_t GPS_ANGLE_DATA_ID = 304;
const uint16_t GPS_ALTITUDE_DATA_ID = 305;
const uint16_t GPS_SATELLITES_DATA_ID = 306;

const uint16_t IMU_TEMP_DATA_ID = 1313;
const uint16_t IMU_ACCEL_X_DATA_ID = 1314;
const uint16_t IMU_ACCEL_Y_DATA_ID = 1315;
const uint16_t IMU_ACCEL_Z_DATA_ID = 1316;
const uint16_t IMU_GYRO_ROLL_DATA_ID = 1317;
const uint16_t IMU_GYRO_PITCH_DATA_ID = 1318;
const uint16_t IMU_GYRO_YAW_DATA_ID = 1319;
const uint16_t IMU_MAG_X_DATA_ID = 1320;
const uint16_t IMU_MAG_Y_DATA_ID = 1321;
const uint16_t IMU_MAG_Z_DATA_ID = 1322;

uint64_t gps_lat_lon = 0;

Adafruit_GPS GPS(&Serial7);
//SoftwareSerial mySerial(3, 2);

void setup()  
{   
  Wire.setModule(0);
  Wire.begin();
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  //Serial.begin(115200);
  
  //connect to roveComm
  Ethernet.enableActivityLed();
  Ethernet.enableLinkLed();
  roveComm_Begin(192,168,1,133);
  //Serial.println("roveComm_Begin");
  
  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  //Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  //Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

}//end 

uint32_t timer = millis();

void loop() 
{
  uint16_t data_id = 0;
  size_t data_size = 0;
  uint16_t data = 0;
  roveComm_GetMsg(&data_id, &data_size, &data);
  //delay(300);
  
  //int16_t msg = 0;
  //roveComm_SendMsg(301, sizeof(msg), &msg);
  //delay(300);
  
  //Serial.print("Looping");
  //delay(1);
  
  char c = GPS.read();
  
  delay(1);
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
  
    if (!GPS.parse(GPS.lastNMEA() ) )// this also sets the newNMEAreceived() flag to false
    {   
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }//end if
    
  }//end if

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) 
  {
    timer = millis();
  }//end if

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
   
    //debug
    /*
    GPS.fix = true;
    GPS.fixquality = 200;
    GPS.latitude_fixed = 407098514;
    GPS.longitude_fixed = -740079168;
    GPS.speed = 123.456;
    GPS.angle = 789.012;
    GPS.altitude = 345.678;
    GPS.satellites = 251;
   */
    if(!GPS.fix)
    {
      GPS.fixquality = 0;
    }//end if
    
    //Serial.print(" quality: "); Serial.println(GPS.fixquality);
    roveComm_SendMsg(GPS_FIX_QUALITY_DATA_ID, sizeof(GPS.fixquality), &GPS.fixquality);
    
    if (GPS.fix) 
    {  
      //TODO: VERIFY ADAFRUIT_GPS PULL #13
      gps_lat_lon = GPS.latitude_fixed;
      gps_lat_lon = gps_lat_lon << 32;
      gps_lat_lon += GPS.longitude_fixed;
      
      roveComm_SendMsg(GPS_LAT_LON_DATA_ID, sizeof(gps_lat_lon), &gps_lat_lon);
      
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      roveComm_SendMsg(GPS_SPEED_DATA_ID, sizeof(GPS.speed), &GPS.speed);
      
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      roveComm_SendMsg(GPS_ANGLE_DATA_ID, sizeof(GPS.angle), &GPS.angle);
      
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      roveComm_SendMsg(GPS_ALTITUDE_DATA_ID, sizeof(GPS.altitude), &GPS.altitude);
      
      //Serial.print("Satellites: "); Serial.println(GPS.satellites);      
      roveComm_SendMsg(GPS_SATELLITES_DATA_ID, sizeof(GPS.satellites), &GPS.satellites);
    }//end if
  }//end if
  
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
  roveComm_SendMsg(IMU_GYRO_PITCH_DATA_ID, sizeof(real_X_Axis), &real_X_Axis);
  roveComm_SendMsg(IMU_GYRO_ROLL_DATA_ID, sizeof(real_Y_Axis), &real_Y_Axis);
  roveComm_SendMsg(IMU_GYRO_YAW_DATA_ID, sizeof(real_Z_Axis), &real_Z_Axis);
                                         
                                              
  //accelerometer/magnetometer section
  byte X_L_M = I2CReceive(Address_M, OUT_X_L_M);//Magnetometer data expressed as two's complement
  byte X_H_M = I2CReceive(Address_M, OUT_X_H_M);
  byte Y_L_M = I2CReceive(Address_M, OUT_Y_L_M);
  byte Y_H_M = I2CReceive(Address_M, OUT_Y_H_M);
  byte Z_L_M = I2CReceive(Address_M, OUT_Z_L_M);
  byte Z_H_M = I2CReceive(Address_M, OUT_Z_H_M);

  byte X_L_A = I2CReceive(Address_AG, OUT_X_L_XL);//Output acceleration in x-axis as a 16-bit word in two's complement
  byte X_H_A = I2CReceive(Address_AG, OUT_X_H_XL);
  byte Y_L_A = I2CReceive(Address_AG, OUT_Y_L_XL);
  byte Y_H_A = I2CReceive(Address_AG, OUT_Y_H_XL);
  byte Z_L_A = I2CReceive(Address_AG, OUT_Z_L_XL);
  byte Z_H_A = I2CReceive(Address_AG, OUT_Z_H_XL);

  int16_t X_AXIS_M = X_H_M <<8 | X_L_M;
  int16_t Y_AXIS_M = Y_H_M <<8 | Y_L_M;
  int16_t Z_AXIS_M = Z_H_M <<8 | Z_L_M;

  int16_t X_AXIS_A = X_H_A <<8 | X_L_A;
  int16_t Y_AXIS_A = Y_H_A <<8 | Y_L_A;
  int16_t Z_AXIS_A = Z_H_A <<8 | Z_L_A;

  //temperature section
  byte Temp_L = I2CReceive(Address_AG, OUT_TEMP_L);
  byte Temp_H = I2CReceive(Address_AG, OUT_TEMP_H);

  int16_t Temp = Temp_H <<8 | Temp_L;
  float real_Temp = (Temp/16.0)+25;
  roveComm_SendMsg(IMU_TEMP_DATA_ID, sizeof(real_Temp), &real_Temp);

  float real_X_Axis_M = X_AXIS_M*0.00014;
  float real_Y_Axis_M = Y_AXIS_M*0.00014;
  float real_Z_Axis_M = Z_AXIS_M*0.00014;
  roveComm_SendMsg(IMU_MAG_X_DATA_ID, sizeof(real_X_Axis_M), &real_X_Axis_M);
  roveComm_SendMsg(IMU_MAG_Y_DATA_ID, sizeof(real_Y_Axis_M), &real_Y_Axis_M);
  roveComm_SendMsg(IMU_MAG_Z_DATA_ID, sizeof(real_Z_Axis_M), &real_Z_Axis_M);

  float real_X_AXIS_A = X_AXIS_A*0.000061;
  float real_Y_AXIS_A = Y_AXIS_A*0.000061;
  float real_Z_AXIS_A = Z_AXIS_A*0.000061;

  roveComm_SendMsg(IMU_ACCEL_X_DATA_ID, sizeof(real_X_AXIS_A), &real_X_AXIS_A);
  roveComm_SendMsg(IMU_ACCEL_Y_DATA_ID, sizeof(real_Y_AXIS_A), &real_Y_AXIS_A);
  roveComm_SendMsg(IMU_ACCEL_Z_DATA_ID, sizeof(real_Z_AXIS_A), &real_Z_AXIS_A);

}//end loop
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

