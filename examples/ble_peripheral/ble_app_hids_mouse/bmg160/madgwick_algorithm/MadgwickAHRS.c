//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	400.0f		// sample frequency in Hz
#define betaDef		0.05f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

volatile int bInitialized = false;

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm intialize

int MadgwickInit(float ax, float ay, float az, float mx, float my, float mz)
{
	if ((mx <= 0.001f) && (my <= 0.001f) && (mz <= 0.001f))
		return false;

	float normA = sqrt(ax*ax + ay*ay + az*az);
	float g = 9.81f;
	if (normA < 0.8f * g) {
		// gravity less than 10% of normal value
		return 0;
	}

	float Hx = my*az - mz*ay;
	float Hy = mz*ax - mx*az;
	float Hz = mx*ay - my*ax;
	float normH = sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
	if (normH < 0.1f) {
		// device is close to free fall (or in space?), or close to
		// magnetic north pole. Typical values are  > 100.
		return 0;
	}

	float invH = 1.0f / normH;
	Hx *= invH;
	Hy *= invH;
	Hz *= invH;

	float invA = 1.0f / normA;
	float Ax = ax * invA;
	float Ay = ay * invA;
	float Az = az * invA;
	
	float Mx = Ay*Hz - Az*Hy;
	float My = Az*Hx - Ax*Hz;
	float Mz = Ax*Hy - Ay*Hx;

	float S = 0.5f / sqrt(1 + Mx*Mx + Hy*Hy + Az*Az);
	q0 = 0.25f / S;
	q1 = ( Ay + Hz ) * S;
	q2 = ( Mz - Ax ) * S;
	q3 = ( -Hx - My ) * S;

	return 1;
}


//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float* data) 
{
	float gx,gy,gz,ax,ay,az,mx,my,mz;
	gx = data[0];
	gy = data[1];
	gz = data[2];
	ax = data[3];
	ay = data[4];
	az = data[5];
	mx = data[6];
	my = data[7];
	mz = data[8];
	
	
//	NRF_LOG_INFO("gyro: %6d, %6d, %6d\n", (int32_t)(gx*1000), (int32_t)(gy*1000), (int32_t)(gz*1000));
//	NRF_LOG_INFO("acc: %6d, %6d, %6d\n", (int32_t)(ax*1000), (int32_t)(ay*1000), (int32_t)(az*1000));
//	NRF_LOG_INFO("mag: %6d, %6d, %6d\n",(int32_t)(mx),(int32_t)(my),(int32_t)(mz));
	
	if (!bInitialized) {
		//bInitialized = MadgwickInit(ax, ay, az, mx, my, mz);
		//return;
	}

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	//NRF_LOG_INFO("------------------------ 12[%d][%d][%d]\n\r",(int32_t)(gx*1000),(int32_t)(gy*1000),(int32_t)(gz*1000));
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx <= 0.001f) && (my <= 0.001f) && (mz <= 0.001f)) {
		//NRF_LOG_INFO("------------------------ no magnet\n\r");
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	//NRF_LOG_INFO("-----------------------step 1- qD0-q3[%d][%d][%d][%d]\n\r",(int32_t)(qDot1*10000),(int32_t)(qDot2*10000),(int32_t)(qDot3*10000),(int32_t)(qDot4*10000));
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax <= 0.001f) && (ay <= 0.001f) && (az <= 0.001f))) {

		//NRF_LOG_INFO("-----------------------step 2- gyro no NULL x[%d] y[%d] z[%d]\n\r", (int32_t)(ax*1000),(int32_t)(ay*1000),(int32_t)(az*1000));
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
	//NRF_LOG_INFO("-----------------------step 3- qD0-q3[%d][%d][%d][%d]\n\r",(int32_t)(qDot1*10000),(int32_t)(qDot2*10000),(int32_t)(qDot3*10000),(int32_t)(qDot4*10000));
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	//NRF_LOG_INFO("----q0-3 *1000000 [%8d][%8d][%8d][%8d]",(int32_t)(q0*1000000),(int32_t)(q1*1000000),(int32_t)(q2*1000000),(int32_t)(q3*1000000));
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
//add
#define  RAD_TO_DEGREE	 (57.29577951289617f)
void QuaternionToDegreeFast(float *DegreeArray)
{
	float RotateMatrixArray[9];
	float mOrientationRadianArray[3];	
	
	float sq_q1 = 2 * q1 * q1;
	float sq_q2 = 2 * q2 * q2;
	
	RotateMatrixArray[1] = 2 * (q1 * q2 - q3 * q0);
	RotateMatrixArray[4] = 1 - sq_q1 - 2 * q3 * q3;	
	RotateMatrixArray[6] = 2 * (q1 * q3 -  q2 * q0);
	RotateMatrixArray[7] = 2 * (q2 * q3 +  q1 * q0);
	RotateMatrixArray[8] = 1 - sq_q1 - sq_q2;
	
	mOrientationRadianArray[0] = (float) atan2f(RotateMatrixArray[1], RotateMatrixArray[4]);
	mOrientationRadianArray[1] = (float) asinf(-RotateMatrixArray[7]);
	mOrientationRadianArray[2] = (float) atan2f(-RotateMatrixArray[6], RotateMatrixArray[8]);

	DegreeArray[0]  = mOrientationRadianArray[0]* RAD_TO_DEGREE ;
	DegreeArray[1]  = mOrientationRadianArray[1]* RAD_TO_DEGREE ;
	DegreeArray[2]  = mOrientationRadianArray[2]* RAD_TO_DEGREE ;

}

void quatToEuler(float* angle)
{
	float q[4];
	q[0]=q0;
	q[1]=q1;
	q[2]=q2;
	q[3]=q3;
  float R32 = 2 * (q[2]*q[3]+q[0]*q[1]);
  float R31 = 2 * (q[1]*q[3]-q[0]*q[2]);
  float R33 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  float R12 = 2 * (q[1]*q[2]-q[0]*q[3]);
  float R22 = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
  
  angle[0] = asin(R32) * RAD_TO_DEGREE;
  angle[1] = atan2( -R31, R33 ) * RAD_TO_DEGREE;
  angle[2] = atan2( -R12, R22 ) * RAD_TO_DEGREE;
}


void quatToEuler1(float *q,float* angle)
{
	float w = q[0],x = q[1],y = q[2],z = q[3];
  
  angle[0] = atan2( 2*(w*x+y*z),1-2*(x*x+y*y) ) * RAD_TO_DEGREE;
	angle[1] = asin( 2*(w*y - z*x) ) * RAD_TO_DEGREE;
  angle[2] = atan2( 2*(w*z+x*y),1-2*(y*y+z*z) ) * RAD_TO_DEGREE;	
}