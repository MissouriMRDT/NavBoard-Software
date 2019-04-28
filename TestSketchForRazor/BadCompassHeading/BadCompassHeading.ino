#include <SparkFunMPU9250-DMP.h>
#include <Time.h>

#define SerialPort SerialUSB
MPU9250_DMP imu;
float hardIron[3] = {-91.5, 180, -185.5};
float softIron[3] = {1.302386, 0.961905, 0.838522};
float accelCalib[3] = {-0.01379, 0.0067, -0.0624};
float gyroCalib[3] = {0.071614, 0.624156, -0.75205};
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
  float temp = imu.heading * 100;
  //temp = temp*180/3.1415;
  int output = (int)temp;
  output = output % 628;
  /*SerialPort.print(output);
  SerialPort.print(",");
  temp = imu.roll * 100;
  output = (int)(temp);
  SerialPort.print(temp);
  SerialPort.print(",");
  temp = imu.pitch * 100;
  output = (int)(temp);
  SerialPort.print(temp);
  SerialPort.print(",");
  SerialPort.println(atan2(imu.magX, imu.magY)*100);*/
  /*SerialPort.print(imu.magX);
  SerialPort.print(",");
  SerialPort.print(imu.magY);
  SerialPort.print(",");
  SerialPort.print(imu.magZ);
  SerialPort.print(",");
  SerialPort.print(imu.heading);
  SerialPort.println("\n");*/
  /*SerialPort.print(output);
  temp = imu.pitch *180/3.14;
  output = (int8_t)temp;
  SerialPort.print(output);
  temp = imu.roll *180/3.14;
  output = (int8_t)temp;*/
  //SerialPort.println(finalOut);
  /*SerialPort.print(imu.magX);
  SerialPort.print(",");
  SerialPort.print(imu.magY);
  SerialPort.print(",");
  SerialPort.println(imu.magZ);*/
  delay(200);
}


void printIMUData()
{
  imu.accelX = imu.ax/(float)imu._aSense;
  imu.accelY = imu.ay/(float)imu._aSense;
  imu.accelZ = -(imu.az)/(float)imu._aSense; //imu._aSense needs to be float-cast as it is an unsigned short, might look into changing to float at some point.
  imu.magX = imu.alpha*((imu.mx)-imu.mag_Bias[0])*imu.mag_Scale[0] + (1.0 - imu.alpha)*imu.magY;
  imu.magY = imu.alpha*((imu.my)-imu.mag_Bias[1])*imu.mag_Scale[1] + (1.0 - imu.alpha)*imu.magX;
  imu.magZ = imu.alpha*((imu.mz)-imu.mag_Bias[2])*imu.mag_Scale[2] + (1.0 - imu.alpha)*imu.magZ;
  imu.currAccelX = imu.accelX * imu.alpha + (imu.currAccelX * (1.0 - imu.alpha));
  imu.currAccelY = imu.accelY * imu.alpha + (imu.currAccelY * (1.0 - imu.alpha));
  imu.currAccelZ = imu.accelZ * imu.alpha + (imu.currAccelZ * (1.0 - imu.alpha));
  imu.roll = asin(imu.currAccelY, imu.currAccelZ);//atan2(imu.currAccelY, sqrt(imu.currAccelX * imu.currAccelX + imu.currAccelZ * imu.currAccelZ));
  imu.pitch = asin(-imu.currAccelX, sqrt(imu.currAccelY * imu.currAccelY + imu.currAccelZ * imu.currAccelZ);//-atan2(imu.currAccelX, sqrt(imu.currAccelY * imu.currAccelY + imu.currAccelZ * imu.currAccelZ));
  Yh = imu.magY * cos(imu.roll) - imu.magZ * sin(imu.roll);
  Xh = imu.magX * cos(imu.pitch) + imu.magY * sin(imu.roll) * sin(imu.pitch) + imu.magZ * cos(imu.roll) * sin(imu.pitch);
  imu.heading = atan2(Yh*100,Xh*100);

}
