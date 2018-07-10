#ifndef __ONLINE_CALIBRATION__
#define __ONLINE_CALIBRATION__



#define _GRAVITY							(9.81f)				// gravity, m/s^2
#define FREQ									400					// sample frequency
#define CHECK_INTERVAL      	((int)(0.2 * FREQ))

#define NOISE									(0.01f)		// acc noise - discrete
#define MAG_VAR0							0.0010f
#define MAG_VAR1							0.0010f
#define MAG_VAR2							0.0050f



typedef struct
{
    int bCalibrated_gyro;
    int bCalibrated_acc;
    int bCalibrated_mag;

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
//int tryCalibration( CalibResult* res,
//                     float ax, float ay, float az,
//                     float gx, float gy, float gz,
//                     float mx, float my, float mz,
//                     float q0, float q1, float q2, float q3,
//                     double timestamp);
										 
int tryCalibration( CalibResult* res, float* imuData, float* q);

// correct acc
void correctAcc(float* acc, float* acc_scale, float* acc_bias);

// correct gyro
void correctGyro(float* gyro, float* gyro_bias);

// correct mag
void correctMag(float* mag, float* mag_scale, float* mag_bias);


#endif
