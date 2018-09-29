#include "Quaternion.h"


    //Class provides sensor fusion allowing heading, pitch and roll to be extracted. This uses the Madgwick algorithm.
    //The update method must be called periodically. The calculations take 1.6mS on the Pyboard.

    float radians(float degrees)
    {
      return ( degrees * 3.14159265359 ) / 180 ;
    }

    float degrees(float radians)
    {
      return ( radians * 180 ) / 3.14159265359 ;
    }

    void Quaternion::init()
    {
      magbias[0] = 0;
	  magbias[1] = 0;
	  magbias[2] = 0;
      q[0] = 1.0;
	  q[1] = 0.0;
	  q[2] = 0.0; 
	  q[3] = 0.0;
      GyroMeasError = radians(600); // adjust this value to adjust between accuracy and speed
      beta = sqrt(3.0 / 4.0) * GyroMeasError;  // compute betas
      pitch = 0;
      heading = 0;
      headingOffset = 0;
      trueHeading = 0;
      roll = 0;
      deltat = .0025;
      magmax[0] =  0.56;
	  magmax[1] =  0.436379998922348;
	  magmax[2] =  0.4745999872684479;
      magmin[0] =  -0.08232999950647354;
	  magmin[1] =  -0.2232999950647354;
	  magmin[2] =  -0.20509999990463257;
      magbias[0] = (magmax[0] + magmin[0])/2;
      magbias[1] = (magmax[1] + magmin[1])/2;
      magbias[2] = (magmax[2] + magmin[2])/2;

      headingOffset = -76;
	}

    void Quaternion::calculateLoop(float gyroscopeXYZ[], float accelerometerXYZ[], float magnetometerXYZ[])    // 3-tuples (x, y, z) for accel, gyro and mag data
	{
            float mx = magnetometerXYZ[0] - magbias[0];
            float my = magnetometerXYZ[1] - magbias[1];
            float mz = magnetometerXYZ[2] - magbias[2];
            float ax = accelerometerXYZ[0];
            float ay = accelerometerXYZ[1];
            float az = accelerometerXYZ[2];
            float gx = radians(gyroscopeXYZ[0]);
            float gy = radians(gyroscopeXYZ[1]);
            float gz = radians(gyroscopeXYZ[2]);
            float q1 = q[0];
            float q2 = q[1];
            float q3 = q[2];
            float q4 = q[3];
            //  Auxiliary variables to avoid repeated arithmetic
            float _2q1 = 2 * q1;
            float _2q2 = 2 * q2;
            float _2q3 = 2 * q3;
            float _2q4 = 2 * q4;
            float _2q1q3 = 2 * q1 * q3;
            float _2q3q4 = 2 * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            float norm = sqrt(ax * ax + ay * ay + az * az);
            if(norm == 0)
            {
                return;  // handle NaN
            }
            norm = 1 / norm;                     // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if(norm == 0)
            {
                return;                          // handle NaN
            }

            norm = 1 / norm;                    // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            float _2q1mx = 2 * q1 * mx;
            float _2q1my = 2 * q1 * my;
            float _2q1mz = 2 * q1 * mz;
            float _2q2mx = 2 * q2 * mx;
            float hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            float hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            float _2bx = sqrt(hx * hx + hy * hy);
            float _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            float _4bx = 2 * _2bx;
            float _4bz = 2 * _2bz;

            // Gradient descent algorithm corrective step
            float s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3
                  * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                  + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                  + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz));

            float s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2
                  * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz
                  * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz
                  * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4)
                                                                     + _2bz * (0.5 - q2q2 - q3q3) - mz));

            float s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3
                  * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz
                                                                                 * (q2q4 - q1q3) - mx)
                  + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                  + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz));

            float s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2)
                  * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3)
                  * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2
                  * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz));

            norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            float qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            float qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            float qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            float qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

            heading = declination + degrees(atan2(2.0 * (q[1] * q[2] + q[0] * q[3]),
                                                            q[0] * q[0] + q[1] * q[1] - q[2]
                                                            * q[2] - q[3] * q[3]));
            trueHeading = fmod((((heading - 360) * -1) - headingOffset), (float)360);

            roll = (float)(-degrees(-asin(2.0 * (q[1] * q[3] - q[0] * q[2]))))+100;
            pitch = pitchAdjust + degrees(atan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                                                          q[0] * q[0] - q[1] * q[1] - q[2]
                                                          * q[2] + q[3] * q[3]));
	}

	float Quaternion::getHeading()
	{
		return heading;
	}

	float Quaternion::getPitch()
	{
		return pitch;
	}

	float Quaternion::getRoll()
	{
		return roll;
	}

	float Quaternion::getTrueHeading()
	{
		return trueHeading;
	}
