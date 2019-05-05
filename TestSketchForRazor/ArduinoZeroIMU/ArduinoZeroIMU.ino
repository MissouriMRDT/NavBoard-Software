#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

#define DEBUG 0

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

float magMax[] = {0,0,0};
float magMin[] = {0,0,0};
int calibration_count = 50;
float heading;
float pitch;
float roll;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() 
{
  if(DEBUG)
  {
    Serial.begin(115200);
  }
  else
  {
    Serial.begin(9600);
  }
  

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
}

void loop() 
{
  static bool first_read = true;
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  if(first_read)
  {
    first_read = false;
    magMax[0] = m.magnetic.x;
    magMax[1] = m.magnetic.y;
    magMax[2] = m.magnetic.z;
    magMin[0] = m.magnetic.x;
    magMin[1] = m.magnetic.y;
    magMin[2] = m.magnetic.z;
  }
  
  if(magMax[0] < m.magnetic.x) magMax[0] = m.magnetic.x;
  if(magMax[1] < m.magnetic.y) magMax[1] = m.magnetic.y;
  if(magMax[2] < m.magnetic.z) magMax[2] = m.magnetic.z;
  if(magMin[0] > m.magnetic.x) magMin[0] = m.magnetic.x;
  if(magMin[1] > m.magnetic.y) magMin[1] = m.magnetic.y;
  if(magMin[2] > m.magnetic.z) magMin[2] = m.magnetic.z;

  if(DEBUG)
  {
    Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
    Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
    Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");
  }
  
  
  m.magnetic.x = mapfloat(m.magnetic.x, magMin[0], magMax[0], -1, 1);
  m.magnetic.y = mapfloat(m.magnetic.y, magMin[1], magMax[1], -1, 1);
  m.magnetic.z = mapfloat(m.magnetic.z, magMin[2], magMax[2], -1, 1);

  if(DEBUG)
  {
    Serial.print("Cal X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
    Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
    Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");
  
    Serial.print("Min X: "); Serial.print(magMin[0]);   Serial.print(" gauss");
    Serial.print("\tY: "); Serial.print(magMin[1]);     Serial.print(" gauss");
    Serial.print("\tZ: "); Serial.print(magMin[2]);     Serial.println(" gauss");
  
    Serial.print("Max X: "); Serial.print(magMax[0]);   Serial.print(" gauss");
    Serial.print("\tY: "); Serial.print(magMax[1]);     Serial.print(" gauss");
    Serial.print("\tZ: "); Serial.print(magMax[2]);     Serial.println(" gauss");
  }
  

  heading = atan2f(m.magnetic.x, m.magnetic.y);
  pitch = atan2f(m.magnetic.x, m.magnetic.z);
  roll = atan2f(m.magnetic.y, m.magnetic.z);
  heading = heading *180/PI;

  heading += 180;
  /*roll = atan2(a.acceleration.y, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));
  pitch = -atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));
  heading = atan2(m.magnetic.y * cos(roll) - m.magnetic.z * sin(roll), m.magnetic.x * cos(pitch) + m.magnetic.y * sin(roll) * sin (pitch) + m.magnetic.z * cos(roll) * sin(pitch));
  heading = heading*180/PI;*/ //this math is bad apparently
/*
  while(Serial.available())
  {
    Serial.print(">>"); Serial.println((char)Serial.read()-'0');
  }
*/
    
  if(DEBUG)
  {
    Serial.print("Heading:"); Serial.println(heading);
  
    Serial.println();
  }

  Serial.println((int)heading);
  
  delay(200);
}
