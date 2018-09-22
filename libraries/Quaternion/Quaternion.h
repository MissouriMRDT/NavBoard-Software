#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Math.h>


using namespace std;
class Quaternion
{
    private:
      int declination = 180;                        // Optional offset for true north. A +ve value adds to heading
      int pitchAdjust = 90;
      float magbias[3];
      float q[4];
      float GyroMeasError;
      float beta;  // compute beta
      float pitch;
      float heading;
      float headingOffset;
      float trueHeading;
      float roll;
      float deltat;
      float magmax[3];
      float magmin[3];

    public:
      void init();
      void calculateLoop(float gyroscopeXYZ[], float accelerometerXYZ[], float magnetometerXYZ[]);
      float getPitch();
      float getRoll();
      float getHeading();
      float getTrueHeading();


};


#endif


