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
    /*
    Start-up process from data sheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bhi160b-ds000.pdf
    1. Power on or reset device
    2. Wait for Interrupt
    3. Upload the Firmware (RAM patch)
    4. Switch into main execution mode
    5. Wait for Interrupt
    6. Configure the sensors and meta events
    7. Configure the FIFO buffers
    7. Configure the host interrupt setting
    */


}



void getICMData()
{
    // ICM-20608-G Documentation: https://invensense.tdk.com/wp-content/uploads/2015/03/DS-000081-v1.01.pdf?ref_disty=digikey

}