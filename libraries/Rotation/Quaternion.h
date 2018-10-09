#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Math.h>


class Quaternion
{
    private:
      const float PI_VAL = 3.14159265359;
  
      int   declination = 180;                        // Optional offset for true north. A +ve value adds to heading
      int   pitchAdjust = 90;
  
      float magbias[3];
      float q[4];
      float GyroMeasError = 0;
      float beta = 0;
      float headingOffset = 0;
      float deltat = 0;
      float magmax[3];
      float magmin[3];
      float pitch = 0;
      float roll = 0;
      float heading = 0;
      float trueHeading = 0;
  
    public:
      void  init();
      void  updateMadgwick(float gyroscopeXYZ[], float accelerometerXYZ[], float magnetometerXYZ[]);
      void  updateMahony(float gyroscopeXYZ[], float accelerometerXYZ[], float magnetometerXYZ[]);
      float getPitch();
      float getRoll();
      float getHeading();
      float getTrueHeading();


};


#endif


