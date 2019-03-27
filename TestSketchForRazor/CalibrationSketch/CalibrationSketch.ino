#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB
MPU9250_DMP imu;

void setup() {
  SerialPort.begin(115200);
  delay(5000);
  imu.begin();
  delay(1000);
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(250);
  imu.setAccelFSR(2);
  imu.setSampleRate(10);
  imu.setCompassSampleRate(10);
  //This sketch does not actually calibrate the sensors,
  //you must use the included excel-style file to perform calculation to derive the calibration numbers.
}

void loop() {
  //calibrateMag(60000); //calibrate the magnetometer for 60s
  SerialPort.print("\n\n\n");
  calibrateAccel(10000); //calibrate the accelerometer for 10s
  SerialPort.print("\n\n\n");
  //calibrateGyro(10000); //calibrate the Gyro for 10s
}

void calibrateMag(uint32_t calibTime)
{
  uint32_t timer = millis();
  while (millis() - timer < calibTime)
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    SerialPort.print("mag,");
    SerialPort.print(imu.mx);
    SerialPort.print(",");
    SerialPort.print(imu.my);
    SerialPort.print(",");
    SerialPort.println(imu.mz);
  }
}

void calibrateAccel(uint32_t calibTime)
{
  uint32_t timer = millis();
  while (millis() - timer < calibTime)
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    SerialPort.print("accel,");
    SerialPort.print(imu.ax);
    SerialPort.print(",");
    SerialPort.print(imu.ay);
    SerialPort.print(",");
    SerialPort.println(imu.az);
  }
}

void calibrateGyro(uint32_t calibTime)
{
  uint32_t timer = millis();
  while (millis() - timer < calibTime)
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    SerialPort.print("gyro,");
    SerialPort.print(imu.gx);
    SerialPort.print(",");
    SerialPort.print(imu.gy);
    SerialPort.print(",");
    SerialPort.println(imu.gz);
  }
}


