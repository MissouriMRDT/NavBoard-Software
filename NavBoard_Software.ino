// NavBoard Software                ///////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2023                        ///////////////////////////////////////////////////////////////////////////////////////////////////////
// Grant Brinker and Brady Davis    ///////////////////////////////////////////////////////////////////////////////////////////////////////
// #RoveSoHard                      ///////////////////////////////////////////////////////////////////////////////////////////////////////

#include "NavBoard_Software.h"
#include <Wire.h>       //I2C library for GPS and BHI

TinyGPS gps;

void gpsdump(TinyGPS &gps);

void setup()
{
    // Pin Settings


    // Communication Setup
    Serial.begin(115200);
    delay(100);
    GPS_SERIAL.begin(GPS_SERIAL_BAUD);
    delay(100);
    Serial.println("begin");

    //RoveComm.begin(RC_NAVBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_NAVBOARD_MAC);
    //delay(100);

    //Telemetry.begin(telemetry, 1500000);
}



void loop()
{
  bool newdata = false;
  unsigned long start = millis();

  // Every 5 seconds we print an update
  while (millis() - start < 5000) 
  {
    if (GPS_SERIAL.available()) 
    {
      char c = GPS_SERIAL.read();
      //Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) 
      {
        newdata = true;
      // break;  // uncomment to print new data immediately!
      }
    }
  }
  
  if (newdata)
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsDump();
    Serial.println("-------------");
    Serial.println();
  }
    //getGPSData();
    //getCompassData();
    //getBHIData();
    //getICMData();
    //telemetry();
}



void telemetry()
{

}



void gpsDump()
{
  static long lat, lon;
  static unsigned long age, date, time, chars;
  static int year;
  byte month, day, hour, minute, second, hundredths;
  static unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
  Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
  Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
  Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
  Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
  Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
  Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}



void getCompassData()
{

}



void getBHIData()
{

}



void getICMData()
{

}