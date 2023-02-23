// NavBoard Software                ///////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2023                        ///////////////////////////////////////////////////////////////////////////////////////////////////////
// Grant Brinker and Brady Davis    ///////////////////////////////////////////////////////////////////////////////////////////////////////
// #RoveSoHard                      ///////////////////////////////////////////////////////////////////////////////////////////////////////

#include "NavBoard_Software.h"
#include <Wire.h>       //I2C library for GPS and BHI



void setup()
{
    // Pin Settings


    // Communication Setup
    Serial.begin(9600);     // might need to change baud rate
    delay(100);

    RoveComm.begin(RC_NAVBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_NAVBOARD_MAC);
    delay(100);

    Telemetry.begin(telemetry, 1500000);
    Serial.println("Started: ");
}



void loop()
{
    getGPSData();
    getCompassData();
    getBHIData();
    getICMData();
    telemetry();
}



void telemetry()
{

}



void getGPSData()
{

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