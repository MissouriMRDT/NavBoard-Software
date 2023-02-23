#pragma once

#include "RoveComm.h"

RoveCommEthernet RoveComm;
rovecomm_packet packet;

//timekeeping variables
IntervalTimer Telemetry;

// declaring Ethernet connection
EthernetServer TCPServer(RC_ROVECOMM_NAVBOARD_PORT);


// Pin definitions
#define GPS_SCL A17
#define GPS_SDA A16
#define GPS_TX  22
#define GPS_RX  23

#define BHI_SDA 10
#define BHI_SCL 11

#define ICM_MOSI    36
#define ICM_MISO    35
#define ICM_CS      37
#define ICM_SCLK    14

#define I2C_LED     28
#define UART_LED    27
#define SPI_LED     26



// Function declarations
void getGPSData();
void getCompassData();
void getBHIData();
void getICMData();
void telemetry();