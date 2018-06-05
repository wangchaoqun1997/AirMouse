#ifndef __ONLINE_CALIBRATION__
#define __ONLINE_CALIBRATION__


#ifdef ANDROID
#include <android/log.h>
#define LOG_TAG "ImuCalibration"
#define  LODI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#else
#include <stdio.h>
#define  LODI(...)  printf(__VA_ARGS__)
#include "bsp.h"
#endif


#define _GRAVITY			(9.81f)				// gravity, m/s^2
#define FREQ				200					// sample frequency
#define CHECK_INTERVAL      ((int)(0.2 * FREQ))

#define NOISE				(0.01f)		// acc noise - discrete


typedef struct
{
    bool bCalibrated_gyro;
    bool bCalibrated_acc;
    bool bCalibrated_mag;

    // gyro_truth = gyro - gyro_bias
    float gyro_bias[3];

    // acc_truth = acc_scale * (acc - acc_bias)
    float acc_scale[3];
    float acc_bias[3];

    // mag_truth = mag_scale * (mag - mag_bias)
    float mag_scale[3];
    float mag_bias[3];


} CalibResult;


/*  Try to Calibrate, if succeed, return true, and store result to CalibResult.
 *  ax, ay, az: value of acc
 *  gx, gy, gz: value of gyro
 *  mx, my, mz: value of mag
 *  q0, q1, q2, q3: quaternion
 *  timestamp, unit: second
 */
bool tryCalibration( CalibResult* res,
                     float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float mx, float my, float mz,
                     float q0, float q1, float q2, float q3,
                     double timestamp);

// correct acc
void correctAcc(float* ax, float* ay, float* az, float* acc_scale, float* acc_bias);

// correct gyro
void correctGyro(float* gx, float* gy, float* gz, float* gyro_bias);

// correct mag
void correctMag(float* mx, float* my, float* mz, float* mag_scale, float* mag_bias);


#endif
