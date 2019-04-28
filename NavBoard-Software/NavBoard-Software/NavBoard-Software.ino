#include "Energia.h"
#include "RoveComm.h"
//#include "roveAttachTimerInterrupt.h"

//#include <Adafruit_GPS.h>
#include "libraries/Adafruit_GPS/Adafruit_GPS.h"
//#include <SoftwareSerial.h>

RoveCommEthernetUdp RoveComm;

#define DIRECTION_SWITCH_PIN    PD_2
#define LF_BUTTON_PIN           PK_2
#define LM_BUTTON_PIN           PK_3
#define LR_BUTTON_PIN           PN_4
#define RF_BUTTON_PIN           PN_5
#define RM_BUTTON_PIN           PP_4
#define RR_BUTTON_PIN           PQ_0

const int BUTTONS[6] = {LF_BUTTON_PIN, LM_BUTTON_PIN, LR_BUTTON_PIN, RF_BUTTON_PIN, RM_BUTTON_PIN, RR_BUTTON_PIN};

//Quaternion fusion;

//LSM90S1 IMU;

uint32_t gpsLatLon[2] = {883454352,883454352};
int16_t finalImuData[3] = {0,0,0}; //we're currently sending as radians instead of degrees.
uint8_t gpsTelemetry[2] = {0,0};
uint32_t gpsLatLast = 0;
uint32_t gpsLonLast = 0;
int lidarDistance = 0;
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
void sendButtonCommands();
void setupButtonCommands();

void setup()
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("Serial begin");
  delay(1000);
  Serial2.begin(9600);
  Serial.println("Serial2 IMU begin");
  delay(1000);
  Serial2.setTimeout(50);
  delay(1000);
  Serial5.begin(19200);
  delay(1000);
  //connect to roveComm
  Ethernet.enableActivityLed();
  Ethernet.enableLinkLed();
  RoveComm.begin(136);//RC_SHIMBLENAVBOARD_FOURTHOCTET);
  //Serial.println("roveComm_Begin");

  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  //fusion.init();
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

  setupButtonCommands();
}//end

uint32_t timer = millis();

void loop()
{
  sendButtonCommands();
  
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
  if (millis() - timer > 250) {
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
    
    //if (GPS.fix)
    //{
      //TODO: VERIFY ADAFRUIT_GPS PULL #13
      if (!(GPS.longitude_fixed < 900000000)) //883454352
      {
        gpsLatLon[0] = GPS.longitude_fixed;
      }
      if (!(GPS.latitude_fixed < 300000000))
      {
        gpsLatLon[1] = GPS.latitude_fixed;
      }
      //gpsLatLon[0] = GPS.longitude_fixed;
      //gpsLatLon[1] = GPS.latitude_fixed;
      Serial.println(gpsLatLon[0]);
      Serial.println(gpsLatLon[1]);
      //Serial.println(GPS.longitude_fixed);
      //Serial.println(GPS.latitude_fixed);
      //RoveComm.write(RC_NAVBOARD_GPSLATLON_DATAID, 2, gpsLatLon);
      
      
      /*Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      roveComm_SendMsg(GPS_SPEED_DATA_ID, sizeof(GPS.speed), &GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      roveComm_SendMsg(GPS_ANGLE_DATA_ID, sizeof(GPS.angle), &GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      roveComm_SendMsg(GPS_ALTITUDE_DATA_ID, sizeof(GPS.altitude), &GPS.altitude);
      Serial.print("Satellites: "); Serial.println(GPS.satellites);
      roveComm_SendMsg(GPS_SATELLITES_DATA_ID, sizeof(GPS.satellites), &GPS.satellites);*/
    //}//end if
    
      gpsTelemetry[0] = GPS.fixquality;
      gpsTelemetry[1] = GPS.satellites;
      RoveComm.write(GPS_FIX_SATELLITES_DATAID, 2, gpsTelemetry);
      RoveComm.write(RC_NAVBOARD_GPSLATLON_DATAID, 2, gpsLatLon);
      gpsLatLast = gpsLatLon[0];
      gpsLonLast = gpsLatLon[1];

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


  if(Serial5.available())
  {
    lidarDistance = Serial5.parseInt();
    Serial5.flush();
    Serial.print("\nLidar: ");
    Serial.println(lidarDistance);
  }
  }//end if
  
  
 
}//end loop

/*void updateIMU()
{
  IMU.read();
  count++;
}*/

void readIMU()
{
  tempHeading = 0;
  bytesToRead = Serial2.available();
  Serial.println(bytesToRead);
  if(bytesToRead > 0)
  {
    delay(100);
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
  finalImuData[1] += 180;
  finalImuData[1] = finalImuData[1]%360;
  //finalImuData[1] = map(
  Serial.println();
  Serial2.flush();
  }
  else
  {
    Serial.println("No IMU Data");
  }
} //end readIMU

void setupButtonCommands()
{
  pinMode(DIRECTION_SWITCH_PIN, INPUT);

  for(int i = 0; i<6; i++)
  {
    pinMode(BUTTONS[i], INPUT);
  }
}

void sendButtonCommands()
{
  bool button_pressed = false;
  int16_t data[6] = {0, 0, 0, 0, 0, 0};
  bool direction = digitalRead(DIRECTION_SWITCH_PIN)? 1 :-1;
  for(int i = 0; i<6; i++)
  {
    if(digitalRead(BUTTONS))
    {
      data[i] = 500*direction;
      button_pressed = true;
    }
  }
  if(button_pressed)
  {
    RoveComm.write(RC_DRIVEBOARD_DRIVEMOTORS_HEADER, data);
  }
}
