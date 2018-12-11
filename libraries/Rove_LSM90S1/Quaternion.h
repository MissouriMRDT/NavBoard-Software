#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Math.h>
#include <Energia.h>

const int GYRO_ERROR_DEG = 3.14159265359 * (40.0f / 180.0f);


class Quaternion
{
  private:
    const float PI_VAL = 3.14159265359;
	  const float DECLINATION = 1.25;
	  int lastUpdate = 0;
  
  //  int   pitchAdjust = 90;
  
  //  float magbias[3];
    
    float GyroMeasError = radians(GYRO_ERROR_DEG);
    float beta = sqrt(3.0 / 4.0) * GyroMeasError;          //Algorithm gain
  //  float headingOffset = 0;
  //  float magmax[3];
  //  float magmin[3];
	  float deltat = .25;
    
	    
  public:
    float q[4] = {1.0, 0.0, 0.0, 0.0};              //Quaternion of sensor frame relative to auxiliary frame
	float pitch = 0;
    float roll = 0;
    float heading = 0;
    float trueHeading = 0; 
	  
    void  init();
    void  updateMadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void  updateMahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	  void	updatePYR();
    float getPitch();
    float getRoll();
    float getHeading();
    float getTrueHeading();
	  
};


#endif