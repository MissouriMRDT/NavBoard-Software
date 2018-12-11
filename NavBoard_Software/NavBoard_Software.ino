#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "RoveBoard.h"
#include "RoveComm.h"
#include "Quaternion.h"
#include "LSM90S1.h"

#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>



const uint16_t GPS_FIX_QUALITY_DATA_ID = 1296;
const uint16_t GPS_LAT_LON_DATA_ID = 1297;
const uint16_t GPS_SPEED_DATA_ID = 1298;
const uint16_t GPS_ANGLE_DATA_ID = 1299;
const uint16_t GPS_ALTITUDE_DATA_ID = 1300;
const uint16_t GPS_SATELLITES_DATA_ID = 1301;

const uint16_t IMU_TEMP_DATA_ID = 1313;
const uint16_t IMU_PITCH_DATA_ID = 1314;//Currently not updated on client side
const uint16_t IMU_ROLL_DATA_ID = 1315; //Currently not updated on client side
const uint16_t IMU_TRUE_HEADING_DATA_ID = 1316;//Currently not updated on client side

Quaternion fusion;

LSM90S1 IMU;

uint64_t gps_lat_lon = 0;

Adafruit_GPS GPS(&Serial6);
//SoftwareSerial mySerial(3, 2);

void calibrateMagnetometer()
{
    float magXmin = 32767;
    float magYmin = 32767;
    float magZmin = 32767;
    float magXmax = -32767;
    float magYmax = -32767;
    float magZmax = -32767;
    float time = micros();
    //hackish way of running this loop for approx. 10 seconds
    while(micros() - time < 10000000)
    {
        float MAG_DATA[3];
        IMU.readMag(MAG_DATA);

        if(MAG_DATA[0] > magXmax){ magXmax = MAG_DATA[0]; } 
        if(MAG_DATA[1] > magYmax){ magYmax = MAG_DATA[1]; }
        if(MAG_DATA[2] > magZmax){ magZmax = MAG_DATA[2]; }

        if(MAG_DATA[0] < magXmin){ magXmin = MAG_DATA[0]; }
        if(MAG_DATA[1] < magYmin){ magYmin = MAG_DATA[1]; }
        if(MAG_DATA[2] < magZmin){ magZmin = MAG_DATA[2]; }
     }
     //outputs to serial monitor, will then need to be manually implemented in Quaternion.cpp
     Serial.print("MagMinX ");
     Serial.print(magXmin);
     Serial.print(  + "\n");
     Serial.print("MagMinY ");
     Serial.print(magYmin);
     Serial.print(  + "\n");
     Serial.print("MagMinZ ");
     Serial.print(magZmin);
     Serial.print(  + "\n");
     Serial.print("MagMaxX ");
     Serial.print(magXmax);
     Serial.print(  + "\n");
     Serial.print("MagMaxY ");
     Serial.print(magYmax);
     Serial.print(  + "\n");
     Serial.print("MagMaxZ ");
     Serial.print(magZmax);
     Serial.print(  + "\n");

}

void setup()
{
  Wire.setModule(0);
  Wire.begin();
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("Serial begin");

  //connect to roveComm
  Ethernet.enableActivityLed();
  Ethernet.enableLinkLed();
  roveComm_Begin(192,168,1,133);
  //Serial.println("roveComm_Begin");

  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  fusion.init();
  //Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  //Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
  IMU.begin();
  delay(10);
  IMU.calibrateMag(10000);
  delay(1000);
  IMU.calibrateGyro(10000);
  IMU.calibrateAccel(1000);
  
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

  //delay(1);

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

  // approximately every 2 milliseconds or so, print out the current stats
  // test this value with varying update frequencies
  uint32_t atimer = millis();
  if (millis() - timer > 200) {
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


  //temperature section
  int16_t temperature;
  IMU.readTemp(temperature);
  roveComm_SendMsg(IMU_TEMP_DATA_ID, sizeof(temperature), &temperature);

  //IMU.print(IMUData);
    
  Serial.println("");
  Serial.println("");
  Serial.println("Updating");
  IMU.read();
  //IMU.printRaw();
  //IMU.printCal();
  //fusion.MadgwickQuaternionUpdate(IMUData.gyro, IMUData.accel, IMUData.mag);
  
  
  Serial.print("Pitch ");
  Serial.print(IMU.getPitch());
  Serial.print(  + "\n");
  Serial.print("Roll ");
  Serial.print(IMU.getRoll());
  Serial.print(  + "\n");
  Serial.print("Heading ");
  Serial.print(IMU.getHeading());
  Serial.print(  + "\n");
  Serial.print("True Heading ");
  Serial.println(IMU.getTrueHeading());

  Serial.print("Q0: ");
  Serial.println(IMU.quaternion.q[0], 15);
  Serial.print("Qx: ");
  Serial.println(IMU.quaternion.q[1], 15);
  Serial.print("Qy: ");
  Serial.println(IMU.quaternion.q[2], 15);
  Serial.print("Qz: ");
  Serial.println(IMU.quaternion.q[3], 15);
  
  //roveComm_SendMsg(IMU_GYRO_DATA_ID, sizeof(GYRO_DATA), GYRO_DATA);
  //roveComm_SendMsg(IMU_ACCEL_DATA_ID, sizeof(ACCEL_DATA), ACCEL_DATA);
  //roveComm_SendMsg(IMU_MAG_DATA_ID, sizeof(MAG_DATA), MAG_DATA);

  }//end if
  
  

}//end loop

