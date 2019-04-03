#include <SparkFunMPU9250-DMP.h>
#include "Quaternion.h"
#define SerialPort SerialUSB


Quaternion quat;
MPU9250_DMP imu;
float ax = 0;
float ay = 0;
float az = 0;
float aSense = 0;
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
  aSense = imu._aSense;
	//someone experimentally found that (ay, az, ax, gy, gz, gx, mx, -mz, my) is best for an MPU9250.
	//*bleepy bleep*, 16.5k or so on both x/y axes, but about 15680 -> - 17760 on Z-axis
	//so add 1040 to all z-axis accelerometer readings before dividing by the sensitivity.
}

void loop() {
  // put your main code here, to run repeatedly:
	imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
	ax = imu.ax/aSense;
	ay = imu.ay/aSense;
	az = (imu.az + 1040)/aSense;
}

