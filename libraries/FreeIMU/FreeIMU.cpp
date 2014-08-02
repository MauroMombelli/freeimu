/*
FreeIMU.cpp - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011-2012 Fabio Varesano <fabio at varesano dot net>

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

#include "FreeIMU.h"

FreeIMU::FreeIMU() {
  
  // initialize quaternion
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
  twoKp = twoKpDef;
  twoKi = twoKiDef;
  integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  lastUpdate = 0;
	last_gyro_update = 0;
  now = 0;
	
	for (size_t i = 0; i<sensor_number;i++){
		last_sensor_update[i] = 0;
	}
}

/**
 * Update the DCM algoritm. Try to call this at fixed frequency for best result
 *
*/
void FreeIMU::update(void) {
  now = micros();
  float sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;
	#if DEBUG_DCM >= __DEBUG_INFO
		DEBUG_PRINT("DCM frequency is ");
		DEBUG_PRINT(sampleFreq);
		DEBUG_PRINT("\n");
	#endif
  
	#ifdef __ESTIMATE_DCM_DURATION
	unsigned long time = micros();
	#endif
	ahrsUpdate(sampleFreq);
	#ifdef __ESTIMATE_DCM_DURATION
	time = micros()-time;
	DEBUG_PRINT(time);
	DEBUG_PRINT("us for DCM update\n");
	#endif
}

/**
 * Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
 * compensation algorithms from Sebastian Madgwick's filter which eliminates the need for a reference
 * direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
 * axis only.
 * 
 * @see: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
*/

void  FreeIMU::ahrsUpdate(float sampleFreq) {
	//first of all new gyro value should be ready
	float gx, gy, gz;
	uint8_t update_number = gyro.getRadiant(&gx, &gy, &gz) - last_gyro_update;
	if (update_number > 0){
		#if DEBUG_DCM >= __DEBUG_WARNING
			if (update_number > 1){
				DEBUG_PRINT("Sensor gyro too fast, ");
				DEBUG_PRINT(update_number);
				DEBUG_PRINT(" update missed\n");
			}
		#endif
	}else{
		/**/
		#if DEBUG_DCM >= __DEBUG_WARNING
		DEBUG_PRINT("Sensor gyro too slow\n");
		#endif
		return;
	}

  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	
	// Auxiliary variables to avoid repeated arithmetic
	float qAux[10]={
		q0 * q0,
		q0 * q1,
		q0 * q2,
		q0 * q3,
		q1 * q1,
		q1 * q2,
		q1 * q3,
		q2 * q2,
		q2 * q3,
		q3 * q3
	};
	
	//utils for cicle
	float versor[3];
	float est_dir[3];
	
	for (size_t i = 0; i < sensor_number; i++){
		update_number = sensors[i].getVersor(versor)-last_sensor_update[i];
		if (update_number > 0){
			#if DEBUG_DCM >= __DEBUG_WARNING
				if (update_number > 1){
					DEBUG_PRINT("Sensor ");
					DEBUG_PRINT(i);
					DEBUG_PRINT(" too fast, ");
					DEBUG_PRINT(update_number);
					DEBUG_PRINT(" update missed\n");
				}
			#endif
			if (versor[0] != 0 && versor[1] != 0 && versor[2] != 0){
				#if DEBUG_DCM >= __DEBUG_INFO
					DEBUG_PRINT("Sensor ");
					DEBUG_PRINT(i);
					DEBUG_PRINT(" versor[");
					DEBUG_PRINT( versor[0] );
					DEBUG_PRINT(",");
					DEBUG_PRINT( versor[1] );
					DEBUG_PRINT(",");
					DEBUG_PRINT( versor[2] );
					DEBUG_PRINT("]\n");
				#endif
				last_sensor_update[i] = update_number;
				
				sensors[i].getEstimateDirection(est_dir, qAux);
				
				halfex += (versor[1] * est_dir[2] - versor[2] * est_dir[1]);
				halfey += (versor[2] * est_dir[0] - versor[0] * est_dir[2]);
				halfez += (versor[0] * est_dir[1] - versor[1] * est_dir[0]);
			}else{
				#if DEBUG_DCM >= __DEBUG_WARNING
					DEBUG_PRINT("Sensor ");
					DEBUG_PRINT(i);
					DEBUG_PRINT(" versor is zero\n");
				#endif
			}
		}else{
			#if DEBUG_DCM >= __DEBUG_WARNING
				DEBUG_PRINT("Sensor ");
				DEBUG_PRINT(i);
				DEBUG_PRINT(" read is old\n");
			#endif
		}
	}

  // Apply feedback only when valid data has been gathered from the sensors
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }else{
		#if DEBUG_DCM >= __DEBUG_INFO
			DEBUG_PRINT("Measured halfex[] is zero ");
		#endif
	}
  
  // Integrate rate of change of quaternion
	float half_period = (0.5f * (1.0f / sampleFreq));
  gx *= half_period;   // pre-multiply common factors
  gy *= half_period;
  gz *= half_period;
	
	float qa, qb, qc;
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  
  // Normalise quaternion
  float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

/**
 * Populates array q with a quaternion representing the IMU orientation with respect to the Earth
 * 
 * @param q the quaternion to populate
*/
void FreeIMU::getQ(float *q){
	q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}


/**
 * Returns the Euler angles in radians defined in the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in radians
*/
void FreeIMU::getEulerRad(float * angles) {
  angles[0] = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0*q0 + 2 * q1 * q1 - 1); // psi
  angles[1] = -asin(2 * q1 * q3 + 2 * q0 * q2); // theta
  angles[2] = atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1); // phi
}


/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in degrees
*/
void FreeIMU::getEuler(float * angles) {
  getEulerRad(angles);
  arr3_rad_to_deg(angles);
}


/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
*/
void FreeIMU::getYawPitchRollRad(float * ypr) {
  float gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  
  ypr[0] = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0*q0 + 2 * q1 * q1 - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
}


/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in degrees between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in degrees
*/
void FreeIMU::getYawPitchRoll(float * ypr) {
  getYawPitchRollRad(ypr);
  arr3_rad_to_deg(ypr);
}


/**
 * Converts a 3 elements array arr of angles expressed in radians into degrees
*/
void arr3_rad_to_deg(float * arr) {
  arr[0] *= 180/M_PI;
  arr[1] *= 180/M_PI;
  arr[2] *= 180/M_PI;
}


/**
 * Fast inverse square root implementation. Compatible both for 32 and 8 bit microcontrollers.
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
float invSqrt(float number) {
  union {
    float f;
    int32_t i;
  } y;

  y.f = number;
  y.i = 0x5f375a86 - (y.i >> 1);
  y.f = y.f * ( 1.5f - ( number * 0.5f * y.f * y.f ) );
  return y.f;
}

/* Old 8bit version. Kept it here only for testing/debugging of the new code above.
float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}
*/




