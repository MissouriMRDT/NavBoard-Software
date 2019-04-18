//#include <SPI.h>
//#include <Ethernet.h>
//#include <EthernetUdp.h>

#include "RoveBoard.h"
#include "RoveComm.h"
#include "Quaternion.h"
#include "LSM90S1.h"
//#include "roveAttachTimerInterrupt.h"

#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

RoveCommEthernetUdp RoveComm;



Quaternion fusion;

LSM90S1 IMU;

uint32_t gpsLatLon[2] = {0,0};
int16_t finalImuData[3] = {0,0,0}; //we're currently sending as radians instead of degrees.

Adafruit_GPS GPS(&Serial7);
//SoftwareSerial mySerial(3, 2);

void updateIMU();
int count = 0;
int heading = 0;
//string readingIMU1 = "";
//string readingIMU2 = "";
char imuData[64] = {};
int bytesToRead = 0;
size_t imuRead = 0;
int imuHeading = 0;
int16_t tempHeading = 0;
void setup()
{
  Wire.setModule(0);
  Wire.begin();
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("Serial begin");
  delay(1000);
  Serial2.begin(115200);
  Serial.println("Serial2 IMU begin");
  delay(1000);
  Serial2.setTimeout(50);

  //connect to roveComm
  Ethernet.enableActivityLed();
  Ethernet.enableLinkLed();
  RoveComm.begin(RC_SHIMBLENAVBOARD_FOURTHOCTET);
  //Serial.println("roveComm_Begin");

  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  fusion.init();
  //Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  //Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
  //IMU.begin();
  delay(10);
  //IMU.calibrateMag(30000);
  delay(2000);
  //IMU.calibrateGyro(10000);
  //IMU.calibrateAccel(1000);

  //roveAttachTimerInterrupt(updateIMU,  T0_A, 100, 1);
  
}//end

uint32_t timer = millis();

void loop()
{
  rovecomm_packet packet;
  packet = RoveComm.read();
  //leaving this with no parsing for now as we don't currently have a reason to communicate to NavBoard
  //delay(300);
  
  //int16_t msg = 0;
  //roveComm_SendMsg(301, sizeof(msg), &msg);
  //delay(300);

  //Serial.print("Looping");
  //delay(1);

  char c = GPS.read();

  delay(10);

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
    //roveComm_SendMsg(GPS_FIX_QUALITY_DATA_ID, sizeof(GPS.fixquality), &GPS.fixquality);

    if (GPS.fix)
    {
      //TODO: VERIFY ADAFRUIT_GPS PULL #13
      gpsLatLon[0] = GPS.latitude_fixed;
      gpsLatLon[1] = GPS.longitude_fixed;
      Serial.println(gpsLatLon[0]);
      Serial.println(gpsLatLon[1]);

      /*Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      roveComm_SendMsg(GPS_SPEED_DATA_ID, sizeof(GPS.speed), &GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      roveComm_SendMsg(GPS_ANGLE_DATA_ID, sizeof(GPS.angle), &GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      roveComm_SendMsg(GPS_ALTITUDE_DATA_ID, sizeof(GPS.altitude), &GPS.altitude);
      Serial.print("Satellites: "); Serial.println(GPS.satellites);
      roveComm_SendMsg(GPS_SATELLITES_DATA_ID, sizeof(GPS.satellites), &GPS.satellites);*/
    }//end if

      RoveComm.write(RC_NAVBOARD_GPSLATLON_DATAID, 2, gpsLatLon);

  //temperature section
  int16_t temperature;
  //IMU.readTemp(temperature);
  //roveComm_SendMsg(IMU_TEMP_DATA_ID, sizeof(temperature), &temperature);

  //IMU.print(IMUData);

  Serial.println("");
  Serial.println("");
  Serial.println("Updating");
  Serial.print("Counts: ");
  Serial.println(count);
  count = 0;
  //IMU.printRaw();
  //IMU.printCal();
  //fusion.MadgwickQuaternionUpdate(IMUData.gyro, IMUData.accel, IMUData.mag);
  readIMU();
  Serial.println(finalImuData[1]);
  
  
  /*Serial.print("Pitch ");
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
  IMU.printRaw();
  IMU.printCal();
  */
 //float heading = IMU.getHeading();
 //heading = -heading;
 //heading += 180;
 heading = imuHeading;
 RoveComm.write(RC_NAVBOARD_IMUPYR_DATAID, 3, finalImuData);
  //roveComm_SendMsg(IMU_TRUE_HEADING_DATA_ID, sizeof(heading), &heading);
  //roveComm_SendMsg(IMU_GYRO_DATA_ID, sizeof(GYRO_DATA), GYRO_DATA);
  //roveComm_SendMsg(IMU_ACCEL_DATA_ID, sizeof(ACCEL_DATA), ACCEL_DATA);
  //roveComm_SendMsg(IMU_MAG_DATA_ID, sizeof(MAG_DATA), MAG_DATA);

  }//end if
  
  

}//end loop

void updateIMU()
{
  IMU.read();
  count++;
}

void readIMU()
{
  tempHeading = 0;
  bytesToRead = Serial2.available();
  Serial.println(bytesToRead);
  if(bytesToRead > 0)
  {
  while(bytesToRead > 0)
  {
    bytesToRead = Serial2.available();
    for (int i = 0; i < 10; i++)
    {
      if(bytesToRead > 0)
      {
        imuData[i] = Serial2.read();
      }
    }
    //tempHeading = Serial.parseInt();
  } //end while
  for (int i =0; i < 3; i++)
  {
    if (imuData[i] >= '0' && imuData[i] <= '9')
    {
      tempHeading *= 10;
      tempHeading += imuData[i] - '0';
    }
  }
  imuHeading = tempHeading;
  finalImuData[1] = tempHeading;
  Serial.println();
  }
  else
  {
    Serial.println("No IMU Data");
  }
} //end readIMU
