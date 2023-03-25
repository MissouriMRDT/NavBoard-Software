#ifndef NAVBOARD_SOFTWARE_H
#define NAVBOARD_SOFTWARE_H

#include "RoveComm.h"
#include <TinyGPS.h>

RoveCommEthernet RoveComm;
rovecomm_packet packet;

//timekeeping variables
IntervalTimer Telemetry;

// declaring Ethernet connection
EthernetServer TCPServer(RC_ROVECOMM_NAVBOARD_PORT);

////// Pin definitions //////
// Compass
#define GPS_SCL             A17
#define GPS_SDA             A16

// GPS
#define GPS_TX              29
#define GPS_RX              28

// BHI
#define BHI_SDA 10
#define BHI_SCL 11

// ICM
#define ICM_MOSI    36
#define ICM_MISO    35
#define ICM_CS      37
#define ICM_SCLK    14

// LEDs
#define I2C_LED     28
#define UART_LED    27
#define SPI_LED     26

////// Variable definitions //////
// GPS
#define GPS_SERIAL_BAUD     9600
#define GPS_SERIAL          Serial7
float roverLatLon[2];

////// Function declarations //////
void gpsDump();
void getGPSData();
void getCompassData();
void getBHIData();
void getICMData();
void telemetry();

#endif