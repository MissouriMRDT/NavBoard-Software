#include <SparkFunMPU9250-DMP.h>

#define SerialPort Serial1
MPU9250_DMP imu;

float Xh = 0;
float Yh = 0;
byte headingB[2] = {0,0};
void setup() {
  SerialPort.begin(115200);
  delay(5000);
  //SerialPort.println("testytest");
  imu.begin();
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(250);
  imu.setAccelFSR(2);
  imu.setSampleRate(10);
  imu.setCompassSampleRate(10);
  //there's weird stuff going on with comparisons, if it looks wrong it probably is. Do it seperately.
  //calibration should be done by printing raw data, and doing math in excel, formulas will be provided at some point.
  /*imu.calibrateMag(15000);
  SerialPort.print(imu.magMin[0]);
  SerialPort.print(",");
  SerialPort.print(imu.magMax[0]);
  SerialPort.print(",");
  SerialPort.print(imu.magMin[1]);
  SerialPort.print(",");
  SerialPort.print(imu.magMax[1]);
  SerialPort.print(",");
  SerialPort.print(imu.magMin[2]);
  SerialPort.print(",");
  SerialPort.print(imu.zMax);
  SerialPort.println(",");
  delay(10000);*/
  //-57,584,-215,380,-1221,-440
  //263.5,82.5,-830.5
  //320.5, 297.5, 390.5
  //39.5349, 12.3781, -124.6062
  //336.166667
  //1.0489, 1.1299, 0.8609
  //2.5394, 2.7518, -4.7870
}

void loop() {
  imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
  printIMUData();
  //SerialPort.print("testytest");
  float temp = (imu.heading + 3*(3.14)) * 100 + 480;
  int output = (int)temp;
  output = output % 628;
  SerialPort.println(output);
  SerialPort.println();
  SerialPort.println();
  /*SerialPort.print(output);
  temp = imu.pitch *180/3.14;
  output = (int8_t)temp;
  SerialPort.print(output);
  temp = imu.roll *180/3.14;
  output = (int8_t)temp;*/
  //SerialPort.println(finalOut);
  delay(200);
}


void printIMUData()
{
  imu.accelX = imu.calcAccel(imu.ax);
  imu.accelY = imu.calcAccel(imu.ay);
  imu.accelZ = -imu.calcAccel(imu.az);
  imu.magX = imu.alpha*((imu.mx/6.665)-imu.mag_Bias[0])*imu.mag_Scale[0] + (1.0 - imu.alpha)*imu.magX;
  imu.magY = imu.alpha*((imu.my/6.665)-imu.mag_Bias[1])*imu.mag_Scale[1] + (1.0 - imu.alpha)*imu.magY;
  imu.magZ = imu.alpha*((imu.mz/6.665)-imu.mag_Bias[2])*imu.mag_Scale[2] + (1.0 - imu.alpha)*imu.magZ;
  imu.currAccelX = imu.accelX * imu.alpha + (imu.currAccelX * (1.0 - imu.alpha));
  imu.currAccelY = imu.accelY * imu.alpha + (imu.currAccelY * (1.0 - imu.alpha));
  imu.currAccelZ = imu.accelZ * imu.alpha + (imu.currAccelZ * (1.0 - imu.alpha));
  imu.roll = abs(atan2(-imu.currAccelY, imu.currAccelZ));
  imu.pitch = abs(atan2(imu.currAccelX, sqrt(imu.currAccelY*imu.currAccelY + imu.currAccelZ*imu.currAccelZ)));
  Yh = imu.magY * cos(imu.roll) + imu.magZ * sin(imu.roll);
  Xh = imu.magX*cos(imu.pitch) + imu.magY*sin(imu.roll)*sin(imu.pitch) - imu.magZ*cos(imu.roll)*sin(imu.pitch);
  imu.heading = atan2(Yh,Xh);

}
