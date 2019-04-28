#include "Energia.h"
#include "RoveComm.h"
//#include "roveAttachTimerInterrupt.h"

#include "src/Adafruit_GPS/Adafruit_GPS.h"
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
  //Serial.println("Serial begin");
  delay(1000);
  Serial2.begin(9600);
  //Serial.println("Serial2 IMU begin");
  delay(1000);
  Serial2.setTimeout(50);
  delay(1000);
  Serial5.begin(19200);
  //connect to roveComm
  Ethernet.enableActivityLed();
  Ethernet.enableLinkLed();
  RoveComm.begin(RC_NAVCAMERABOARD_FOURTHOCTET);
  delay(1000);
  //Serial.println("roveComm_Begin");

  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  //Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  setupButtonCommands();
}//end

uint32_t timer = millis();

void loop()
{
  sendButtonCommands();
  
  rovecomm_packet packet;
  packet = RoveComm.read();
  //leaving this with no parsing for now as we don't currently have a reason to communicate to NavBoard
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
  uint32_t atimer = millis();
  if (millis() - timer > 250) {
    timer = millis(); // reset the timer

    if(!GPS.fix)
    {
      GPS.fixquality = 0;
    }//end if
    
    //if (GPS.fix)
    //{
      //TODO: VERIFY ADAFRUIT_GPS PULL #13
      if (!(GPS.longitude_fixed < 900000000) || !(GPS.longitude_fixed > 930000000)) //require longitude to be in Missouri-ish
      {
        gpsLatLon[0] = GPS.longitude_fixed;
      }
      if (!(GPS.latitude_fixed < 300000000) || !(GPS.latitude_fixed > 330000000)) //require latitude to be in Missouri-ish
      {
        gpsLatLon[1] = GPS.latitude_fixed;
      }
      //gpsLatLon[0] = GPS.longitude_fixed;
      //gpsLatLon[1] = GPS.latitude_fixed;
      //Serial.println(gpsLatLon[0]);
      //Serial.println(gpsLatLon[1]);
      //Serial.println(GPS.longitude_fixed);
      //Serial.println(GPS.latitude_fixed);
      //RoveComm.write(RC_NAVBOARD_GPSLATLON_DATAID, 2, gpsLatLon); //was having issues with the software fix flag even though hardware was good. Try NavBoardRev3
    //}//end if
    
  gpsTelemetry[0] = GPS.fixquality;
  gpsTelemetry[1] = GPS.satellites;
  RoveComm.write(RC_NAVBOARD_GPSADD_DATAID, 2, gpsTelemetry);
  RoveComm.write(RC_NAVBOARD_GPSLATLON_DATAID, 2, gpsLatLon);
  gpsLatLast = gpsLatLon[0];
  gpsLonLast = gpsLatLon[1];

  readIMU();
  //Serial.println(finalImuData[1]);
  RoveComm.write(RC_NAVBOARD_IMUPYR_DATAID, 3, finalImuData);

  if(Serial5.available())
  {
    lidarDistance = Serial5.parseInt();
    Serial5.flush();
    //Serial.print("\nLidar: ");
    //Serial.println(lidarDistance);
  }
  }//end loop with delay
  
}//end loop

void readIMU()
{
  tempHeading = 0;
  bytesToRead = Serial2.available();
  //Serial.println(bytesToRead);
  if(bytesToRead > 0)
  {
    delay(100); //test smaller values to improve turn-around time for autonomy
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
  finalImuData[1] = map(finalImuData[1], 0, 359, 359, 0); //flip the data to be clockwise instead of counter clockwise
  //Serial.println();
  Serial2.flush();
  }
  else
  {
    //Serial.println("No IMU Data");
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
    if(digitalRead(BUTTONS[i]))
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
