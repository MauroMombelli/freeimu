/*
FreeIMU.h - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011 Fabio Varesano <fabio at varesano dot net>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

***************************************

This code have been heavily rewritten by 
Mauro Mombelli <mombelli dot mauro at gmail dot com> 
in a stormy night between 2014-08-01 and 2014-08-02

*/

#ifndef FreeIMU_h
#define FreeIMU_h

#include <inttypes.h>
#include <stdint.h>
#include "Arduino.h"
#include "sensor_list.h"

#define __DEBUG_NOTHING 0
#define __DEBUG_WARNING 1
#define __DEBUG_INFO 2


//#define __ESTIMATE_DCM_DURATION
#define DEBUG_DCM __DEBUG_NOTHING

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

class FreeIMU
{
  public:
    FreeIMU();
		void update(void);
    void getQ(float * q);
    void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
    void getEulerRad(float * angles);
    void getYawPitchRollRad(float * ypr);
  private:
    void  ahrsUpdate(float);
	
    //float q0, q1, q2, q3; // quaternion elements representing the estimated orientation
    //float iq0, iq1, iq2, iq3;
		float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    float exInt, eyInt, ezInt;  // scaled integral error
    float twoKp;      // 2 * proportional gain (Kp)
    float twoKi;      // 2 * integral gain (Ki)
    float integralFBx,  integralFBy, integralFBz;
    unsigned long lastUpdate, now; // sample period expressed in milliseconds
    uint8_t last_gyro_update;
		uint8_t last_sensor_update[sensor_number];
};

float invSqrt(float number);
void arr3_rad_to_deg(float * arr);



#endif // FreeIMU_h

