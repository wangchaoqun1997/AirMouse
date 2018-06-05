
#include "IMUCalibration.h"
#include <math.h>



#define USE_EIGEN			0
#if USE_EIGEN
#include "Eigen/Dense"
#endif


#define DEBUG_PRINT			0



typedef struct 
{
	int startIndex;
	int endIndex;

	float gyro[3];
	float acc[3];

} StaticInterval;

void resetInterval(StaticInterval* interval, int startIndex) 
{
	int i;

	interval->startIndex = startIndex;
	for (i = 0; i < 3; i++) {
		interval->acc[i] = 0;
		interval->gyro[i] = 0;
	}
}


static int magArray[26];
static float vMags[26][3];
static int vmags_size; 
static double firstMagTime;
#define MAG_NUM		20
static float H_mag[MAG_NUM][6]; 
static float z_mag[MAG_NUM]; 
static float J_mag[6][6];
static float Jinv_mag[6][6];
static float b_mag[6];


static int accArray[6];
static StaticInterval vStaticIntervals[6];
static int vinterval_size;
static double firstAccTime; 
static float H_acc[6][6];
static float z_acc[6];
static float J_acc[6][6];
static float Jinv_acc[6][6];
static float b_acc[6];




//------------------------- Functions Declarations -------------------------------------------------


int matrix_inv(float a[][6], int n, float at[][6]);
int matrix_t_mul(float a[][6], float b[][6], float c[][6], int m, int n, int l);
int matrix_mul_vector(float a[][6], float* b, float* c, int m, int n);
int matrix_t_mul_vector(float a[][6], float* b, float* c, int m, int n);


void resetMagCalibration();
void resetAccCalibration();
bool checkMagCalibration(float* mag, float* q, double timestamp);
int checkGyroAndAccCalibration(float* gyro, float* acc, double timestamp, float* gyro_bias);
bool calibrateMag(float* mag_scale, float* mag_bias);
bool calibrateAcc(float* acc_scale, float* acc_bias);


//------------------------- Functions Implementations ----------------------------------------------
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern float dof3_buf[6];
extern int magnet_xyz[3];
CalibResult cal_res;

bool tryCalibration(CalibResult* res,
					float ax, float ay, float az,
					float gx, float gy, float gz,
					float mx, float my, float mz,
					float q0, float q1, float q2, float q3,
					double timestamp)
{
	gx = dof3_buf[1];
	gy = dof3_buf[0];
	gz = dof3_buf[2];
	ax = dof3_buf[4];
	ay = dof3_buf[3];
	az = dof3_buf[5];
#if NO_MAG
	mx = 0.0001f;
	my = 0.0001f;
	mz = 0.0001f;
#else
	mx = magnet_xyz[0];
	my = magnet_xyz[1];
	mz = magnet_xyz[2];
#endif


	res->bCalibrated_acc = false;
	res->bCalibrated_gyro = false;
	res->bCalibrated_mag = false;

	float q[] = {q0, q1, q2, q3};
	float mag[] = {mx, my, mz};
	float acc[] = {ax, ay, az};
	float gyro[] = {gx, gy, gz};

	// check mag calibration
	if (checkMagCalibration(mag, q, timestamp)) {
		float mag_scale[3], mag_bias[3];
		if (calibrateMag(mag_scale, mag_bias)) {
			res->bCalibrated_mag = true;

			res->mag_scale[0] = mag_scale[0];
			res->mag_scale[1] = mag_scale[1];
			res->mag_scale[2] = mag_scale[2];

			res->mag_bias[0] = mag_bias[0];
			res->mag_bias[1] = mag_bias[1];
			res->mag_bias[2] = mag_bias[2];
		}
		resetMagCalibration();
	}


	// check gyro and acc calibration
	float gyro_bias[3];
	int ret = checkGyroAndAccCalibration(gyro, acc, timestamp, gyro_bias);
	if (ret == 1) {		// gyro calibrated
		res->bCalibrated_gyro = true;

		res->gyro_bias[0] = gyro_bias[0];
		res->gyro_bias[1] = gyro_bias[1];
		res->gyro_bias[2] = gyro_bias[2];
	}
	else if (ret == 2) {	// need to calibrate acc
		float acc_scale[3], acc_bias[3];
		if (calibrateAcc(acc_scale, acc_bias)) {
			res->bCalibrated_acc = true;

			res->acc_scale[0] = acc_scale[0];
			res->acc_scale[1] = acc_scale[1];
			res->acc_scale[2] = acc_scale[2];

			res->acc_bias[0] = acc_bias[0];
			res->acc_bias[1] = acc_bias[1];
			res->acc_bias[2] = acc_bias[2];
		}
		resetAccCalibration();
	}

	return (res->bCalibrated_acc || res->bCalibrated_gyro || res->bCalibrated_mag);
}



// correct acc
void correctAcc(float* ax, float* ay, float* az, float* acc_scale, float* acc_bias)
{
	*ax = acc_scale[0] * (*ax - acc_bias[0]);
	*ay = acc_scale[1] * (*ay - acc_bias[1]);
	*az = acc_scale[2] * (*az - acc_bias[2]);
}

// correct gyro
void correctGyro(float* gx, float* gy, float* gz, float* gyro_bias)
{
	*gx = *gx - gyro_bias[0];
	*gy = *gy - gyro_bias[1];
	*gz = *gz - gyro_bias[2];
}

// correct mag
void correctMag(float* mx, float* my, float* mz, float* mag_scale, float* mag_bias)
{
	*mx = mag_scale[0] * (*mx - mag_bias[0]);
	*my = mag_scale[1] * (*my - mag_bias[1]);
	*mz = mag_scale[2] * (*mz - mag_bias[2]);
}



void resetMagCalibration()
{
	int i;
	for (i = 0; i < 26; i++) {
		magArray[i] = 0;
	}
	
	vmags_size = 0;
}

void resetAccCalibration()
{
	int i;
	for (i = 0; i < 6; i++) {
		accArray[i] = 0;
	}

	vinterval_size = 0; 
}



int checkMagDirection(float mx, float my, float mz)
{
	if (mx > 0.94f) return 4;
	if (mx < -0.94f) return 21;
	if (my > 0.94f) return 13;
	if (my < -0.94f) return 12;
	if (mz > 0.94f) return 10;
	if (mz < -0.94f) return 15;

	if (mx > 0.4f) {
		if (my > 0.4f) {
			if (mz > 0.4f)
				return 2;
			else if (mz < -0.4f)
				return 8;
			else if (mz < 0.35f && mz > -0.35f)
				return 5;
		}
		else if (my < -0.4f) {
			if (mz > 0.4f)
				return 0;
			else if (mz < -0.4f)
				return 6;
			else if (mz < 0.35f && mz > -0.35f)
				return 3;
		}
		else if (my < 0.35f && my > -0.35f) {
			if (mz > 0.4f)
				return 1;
			else if (mz < -0.4f)
				return 7;
		}
	}
	else if (mx < -0.4f) {
		if (my > 0.4f) {
			if (mz > 0.4f)
				return 19;
			else if (mz < -0.4f)
				return 25;
			else if (mz < 0.35f && mz > -0.35f)
				return 22;
		}
		else if (my < -0.4f) {
			if (mz > 0.4f)
				return 17;
			else if (mz < -0.4f)
				return 23;
			else if (mz < 0.35f && mz > -0.35f)
				return 20;
		}
		else if (my < 0.35f && my > -0.35f) {
			if (mz > 0.4f)
				return 18;
			else if (mz < -0.4f)
				return 24;
		}
	}
	else if (mx < 0.35f && mx > -0.35f){
		if (my > 0.4f) {
			if (mz > 0.4f)
				return 11;
			else if (mz < -0.4f)
				return 16;
		}
		else if (my < -0.4f) {
			if (mz > 0.4f)
				return 9;
			else if (mz < -0.4f)
				return 14;
		}
	}

	return -1;
}


int checkAccDirection(float ax, float ay, float az)
{

    if (ax > 9.22f) return 0;
	if (ax < -9.22f) return 1;
	if (ay > 9.22f) return 2;
	if (ay < -9.22f) return 3;
	if (az > 9.22f) return 4;
	if (az < -9.22f) return 5;


	return -1;
}



bool checkMagCalibration(float* mag, float* q, double timestamp)
{
	int i;
	static unsigned int cnt = 0;

	static float mag_sum[3];
	static float mag_sum2[3];

	static int big_cnt = 0;
	static int small_cnt = 0;
	static int large_cnt = 0;
	static int large_lost_cnt = 0;
	static bool bEightWave = false;

	// first time reset
	static bool bFirstTime = true;
	if (bFirstTime) {
		bFirstTime = false;

		for (i = 0; i < 3; i++) {
			mag_sum[i] = 0;
			mag_sum2[i] = 0;
		}
	}

	for (i = 0; i < 3; i++) {
		mag_sum[i] += mag[i];
		mag_sum2[i] += mag[i] * mag[i];
	}

	cnt++;
	if (bEightWave && (cnt & 3) == 0) {

		float qw = q[0], qx = q[1], qy = q[2], qz = q[3]; 
		int nDir = checkMagDirection(qw*qw + qx*qx - qy*qy - qz*qz,
			2 * qx*qy + 2 * qw*qz, 2 * qx*qz - 2 * qw*qy);
		//LODI("nDir: %d\n", nDir);
		if (nDir >= 0) {
			// check time
			if (vmags_size == 0) {
				firstMagTime = timestamp;
			}
			else {
				if (timestamp - firstMagTime > 60) {
					resetMagCalibration();
					//LODI("resetMagCalibration\n");
				}
			}

			if (magArray[nDir] == 0) {
				magArray[nDir] = 1;

				for (i = 0; i < 3; i++) {
					vMags[vmags_size][i] = mag[i]; 
				}
				vmags_size++; 
				if (vmags_size >= MAG_NUM) {
					return true;
				}
			}
		}
	}

	if (cnt % CHECK_INTERVAL == 0) {
		// standard variance
		// mag variance
		float mag_var[3]; 
		for (i = 0; i < 3; i++) {
			mag_var[i] = (mag_sum2[i] - mag_sum[i] * mag_sum[i] / CHECK_INTERVAL) / CHECK_INTERVAL;
		}
		//LODI("mag_var: %f, %f, %f\n", mag_var[0], mag_var[1], mag_var[2]);
		if ((mag_var[0] > 0.001f && mag_var[1] > 0.001f) ||
			(mag_var[1] > 0.001f && mag_var[2] > 0.001f) ||
			(mag_var[2] > 0.001f && mag_var[0] > 0.001f))
		{
			big_cnt++;
			small_cnt = 0;

			//LODI("8 wave!!!\n");
			if (mag_var[0] > 0.001f && mag_var[1] > 0.001f && mag_var[2] > 0.001f) {
				large_cnt++;
				large_lost_cnt = 0;
			}
			else {
				large_cnt = 0;
				large_lost_cnt++;
			}
		}
		else {
			big_cnt = 0;
			large_cnt = 0;
			small_cnt++;
			large_lost_cnt++;
		}

		if (bEightWave) {
			if (small_cnt >= 4 || large_lost_cnt >= 10) {
				bEightWave = false;
			}
		}
		else {
			if (big_cnt >= 3 && large_cnt >= 1) {
				bEightWave = true;
			}
		}
#if DEBUG_PRINT
        //LODI("bEightWave: %d\n", bEightWave);
#endif

		for (i = 0; i < 3; i++) {
			mag_sum[i] = 0;
			mag_sum2[i] = 0;
		}
	}

	return false;
}


int checkGyroAndAccCalibration(float* gyro, float* acc, double timestamp, float* gyro_bias)
{
	int i;
	int retval = 0;
	static StaticInterval staticInterval;

	static float acc_sum[3];
	static float acc_sum2[3];
	static float gyro_sum[3];

	static unsigned int cnt = 0;

	// first time reset
	static bool bFirstTime = true;
	if (bFirstTime) {
		bFirstTime = false;

		resetInterval(&staticInterval, cnt); 

		for (i = 0; i < 3; i++) {
			acc_sum[i] = 0;
			acc_sum2[i] = 0;
			gyro_sum[i] = 0;
		}
	}

	for (i = 0; i < 3; i++) {
		acc_sum[i] += acc[i];
		acc_sum2[i] += acc[i] * acc[i];
		gyro_sum[i] += gyro[i];
	}

	cnt++;
	if (cnt % CHECK_INTERVAL == 0) {
		// standard variance
		// acc variance
		float acc_var[3]; 
		for (i = 0; i < 3; i++) {
			acc_var[i] = (acc_sum2[i] - acc_sum[i] * acc_sum[i] / CHECK_INTERVAL) / CHECK_INTERVAL;
		}
		//LODI("acc_var: %f\n", acc_var.sum());

		if (acc_var[0] + acc_var[1] + acc_var[2] < NOISE) {	// static
			//LODI("Staic! Standard Variance: %f\n", acc_var.sum());

			// static interval continue
			for (i = 0; i < 3; i++) {
				staticInterval.acc[i] += acc_sum[i];
				staticInterval.gyro[i] += gyro_sum[i];
			}
			staticInterval.endIndex = cnt;

			int num = staticInterval.endIndex - staticInterval.startIndex + 1;
			// first static interval should be larger than INIT_STATIC_INTERVAL
			int N = 1 * FREQ;
			if (num > N) {
				for (i = 0; i < 3; i++) {
					staticInterval.acc[i] /= num;
					staticInterval.gyro[i] /= num;
				}

				for (i = 0; i < 3; i++) {
					gyro_bias[i] = staticInterval.gyro[i];
				}
				retval = 1;
#if DEBUG_PRINT
                LODI("gyro bias: %f, %f, %f\n", staticInterval.gyro[0], staticInterval.gyro[1], staticInterval.gyro[2]);
#endif

				static int last_dir = -1;
				int dir = checkAccDirection(staticInterval.acc[0], staticInterval.acc[1], staticInterval.acc[2]);
				if (dir >= 0 && dir != last_dir) {
					// check time
					if (vinterval_size == 0) {
						firstAccTime = timestamp;
					}
					else {
						if (timestamp - firstAccTime > 300) {
							resetAccCalibration();
							//LODI("resetAccCalibration\n");
						}
					}

					if (accArray[dir] < 1) {
						accArray[dir]++;

						vStaticIntervals[vinterval_size].startIndex = staticInterval.startIndex;
						vStaticIntervals[vinterval_size].endIndex = staticInterval.endIndex;
						for (i = 0; i < 3; i++) {
							vStaticIntervals[vinterval_size].acc[i] = staticInterval.acc[i];
							vStaticIntervals[vinterval_size].gyro[i] = staticInterval.gyro[i];
						}
						vinterval_size++; 
						
						resetInterval(&staticInterval, cnt+1);
#if DEBUG_PRINT
                        LODI("StaticInterval num: %d\n", vinterval_size);
#endif
					}
				}
				last_dir = dir;

				// clear
				resetInterval(&staticInterval, cnt + 1);
			}
		}
		else {					// dynamic
			// clear staticInterval
			resetInterval(&staticInterval, cnt + 1);
		}

		for (i = 0; i < 3; i++) {
			acc_sum[i] = 0;
			acc_sum2[i] = 0;
			gyro_sum[i] = 0;
		}
	}


	if (vinterval_size >= 6)
		retval = 2;
	
	return retval; 
}



bool calibrateAcc(float* acc_scale, float* acc_bias)
{
	int i, j;
	int n = vinterval_size;
	if (n < 6) return false;

#if USE_EIGEN
	Eigen::MatrixXf H(n, 6);
	Eigen::VectorXf z(n);

	for (i = 0; i < vinterval_size; i++) {
		float ax = vStaticIntervals[i].acc[0];
		float ay = vStaticIntervals[i].acc[1];
		float az = vStaticIntervals[i].acc[2];

		H(i, 0) = -2 * ax;
		H(i, 1) = -2 * ay;
		H(i, 2) = -2 * az;
		H(i, 3) = ax * ax;
		H(i, 4) = ay * ay;
		H(i, 5) = 1;
		z(i) = -az * az;
	}
	Eigen::VectorXf x = (H.transpose() * H).inverse() * H.transpose() * z;
#else
	for (i = 0; i < vinterval_size; i++) {
		float ax = vStaticIntervals[i].acc[0];
		float ay = vStaticIntervals[i].acc[1];
		float az = vStaticIntervals[i].acc[2];

		H_acc[i][0] = -2 * ax;
		H_acc[i][1] = -2 * ay;
		H_acc[i][2] = -2 * az;
		H_acc[i][3] = ax * ax;
		H_acc[i][4] = ay * ay;
		H_acc[i][5] = 1;
		z_acc[i] = -az * az;
	}

	float x[6];
	matrix_t_mul(H_acc, H_acc, J_acc, 6, 6, 6);
	matrix_inv(J_acc, 6, Jinv_acc);
	matrix_t_mul_vector(H_acc, z_acc, b_acc, 6, 6);
	matrix_mul_vector(Jinv_acc, b_acc, x, 6, 6);
#endif

	if (x[3] <= 0 || x[4] <= 0) return false; 
	float sx_2 = 1 / x[3];
	float sy_2 = 1 / x[4];

	float bx = x[0] / x[3];
	float by = x[1] / x[4];
	float bz = x[2];

	float sz2 = (_GRAVITY * _GRAVITY) / (x[0] * bx + x[1] * by + x[2] * bz - x[5]);
	if (sz2 <= 0) return false;
	float sx2 = sz2 * x[3];
	float sy2 = sz2 * x[4];

	/// Scale factor
	acc_scale[0] = sqrtf(sx2);
	acc_scale[1] = sqrtf(sy2);
	acc_scale[2] = sqrtf(sz2);

	/// Bias
	acc_bias[0] = bx;
	acc_bias[1] = by;
	acc_bias[2] = bz;

	// show results
#if DEBUG_PRINT
    LODI("\n============================= Acc Calibration ==================================\n");
    LODI("Scale:\n %f, %f, %f\n", acc_scale[0], acc_scale[1], acc_scale[2]);
    LODI("Bias:\n %f, %f, %f\n", acc_bias[0], acc_bias[1], acc_bias[2]);

    LODI("\nCalibrated norm: \n");

	float sample_sum = 0;
	float sample_sum2 = 0;
	for (i = 0; i < vinterval_size; i++)
	{
		float sample_norm = 0;
		for (j = 0; j < 3; j++) {
			float sample = acc_scale[j] * (vStaticIntervals[i].acc[j] - acc_bias[j]);
			sample_norm += sample * sample;
		}
		sample_norm = sqrtf(sample_norm);
        LODI("%f ", sample_norm);
		sample_sum += sample_norm;
		sample_sum2 += sample_norm * sample_norm;
	}
    LODI("\n");

	float mean = sample_sum / vinterval_size;
	float stdVar = sqrtf((sample_sum2 - sample_sum * sample_sum / vinterval_size) / vinterval_size);
    LODI("Mean and Standard Variance:\n %f, %f\n", mean, stdVar);
#endif

	// check result
	for (i = 0; i < 3; i++) {
		if (abs(acc_bias[i]) > 0.3f)
			return false;
		if (abs(acc_scale[i] - 1) > 0.1f)
			return false;
	}

	//// reset calibration
	//resetAccCalibration(); 

	return true;
}



bool calibrateMag(float* mag_scale, float* mag_bias)
{
	int i, j;

	//// save data
	//FILE* pfile = fopen("mag.txt", "w");
	//for (auto& mag : vMags) {
	//	fprintf(pfile, "%f %f %f\n", mag(0), mag(1), mag(2)); 
	//}
	//fclose(pfile);

	int n = vmags_size;

#if USE_EIGEN
	Eigen::MatrixXf H(n, 6);
	Eigen::VectorXf z(n);

	for (i = 0; i < vmags_size; i++) {
		float mx = vMags[i][0];
		float my = vMags[i][1];
		float mz = vMags[i][2];

		H(i, 0) = -2 * mx;
		H(i, 1) = -2 * my;
		H(i, 2) = -2 * mz;
		H(i, 3) = mx * mx;
		H(i, 4) = my * my;
		H(i, 5) = 1;
		z(i) = -mz * mz;
	}
	Eigen::VectorXf x = (H.transpose() * H).inverse() * H.transpose() * z;
#else

	for (i = 0; i < vmags_size; i++) {
		float mx = vMags[i][0];
		float my = vMags[i][1];
		float mz = vMags[i][2];

		H_mag[i][0] = -2 * mx;
		H_mag[i][1] = -2 * my;
		H_mag[i][2] = -2 * mz;
		H_mag[i][3] = mx * mx;
		H_mag[i][4] = my * my;
		H_mag[i][5] = 1;
		z_mag[i] = -mz * mz;
	}

	float x[6];
	matrix_t_mul(H_mag, H_mag, J_mag, 6, MAG_NUM, 6);
	matrix_inv(J_mag, 6, Jinv_mag);
	matrix_t_mul_vector(H_mag, z_mag, b_mag, 6, MAG_NUM);
	matrix_mul_vector(Jinv_mag, b_mag, x, 6, 6);
#endif

	float sx2 = x[3];
	float sy2 = x[4];
	if (sx2 <= 0 || sy2 <= 0) {
		return false;
	}

	/// Scale factor
	mag_scale[0] = sqrtf(sx2);
	mag_scale[1] = sqrtf(sy2);
	mag_scale[2] = 1.0f;

	/// Bias
	mag_bias[0] = x[0] / sx2;
	mag_bias[1] = x[1] / sy2;
	mag_bias[2] = x[2];


	// show results
#if DEBUG_PRINT
    LODI("============================= Mag Calibration ==================================\n");
    LODI("Scale: \n%f, %f, %f\n", mag_scale[0], mag_scale[1], mag_scale[2]);
    LODI("Bias: \n%f, %f, %f\n", mag_bias[0], mag_bias[1], mag_bias[2]);
    LODI("\n");

	float sample_sum = 0;
	float sample_sum2 = 0;
	for (i = 0; i < vmags_size; i++)
	{
		float mag_norm = 0;
		for (j = 0; j < 3; j++) {
			float sample = mag_scale[j] * (vMags[i][j] - mag_bias[j]);
			mag_norm += sample * sample;
		}
		mag_norm = sqrtf(mag_norm);

		sample_sum += mag_norm;
		sample_sum2 += mag_norm * mag_norm;
	}

	float mean = sample_sum / vmags_size;
	float stdVar = sqrtf((sample_sum2 - sample_sum * sample_sum / vmags_size) / vmags_size);

    LODI("Mean and Standard Variance: \n%f %f\n", mean, stdVar);
#endif

	// check result
	for (i = 0; i < 3; i++) {
		if (abs(mag_bias[i]) > 0.8f)
			return false;
		if (abs(mag_scale[i] - 1) > 0.2f)
			return false;
	}

	//// reset calibration
	//resetMagCalibration();

	return true;
}




//-------------------------------- Matrix functions ---------------------------
int cholesky(float orig[][6], int n, float chol[][6])
{
	int i, j, k, l;
	int retval = 1;

	for (i = 0; i < n; i++) {
		chol[i][i] = orig[i][i];
		for (k = 0; k<i; k++)
			chol[i][i] -= chol[k][i] * chol[k][i];
		//if (chol[i][i] <= 0) {
		//	//fprintf(stderr, "\nERROR: non-positive definite matrix!\n");
		//	LODI("\nproblem from %d %f\n", i, chol[i][i]);
		//	retval = 0;
		//	return retval;
		//}
		chol[i][i] = sqrtf(chol[i][i]);

		for (j = i + 1; j < n; j++) {
			chol[i][j] = orig[i][j];
			for (k = 0; k<i; k++)
				chol[i][j] -= chol[k][i] * chol[k][j];
			chol[i][j] /= chol[i][i];
		}
	}

	return retval;
}


int invutri(float u[][6], int n, float ut[][6])
{
	int i, j, k;
	float aii;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			ut[i][j] = i == j ? 1 : 0;
		}
	}

	for (i = n - 1; i >= 0; i--) {
		aii = 1 / u[i][i];
		for (j = i; j < n; j++) {
			ut[i][j] *= aii;
		}
		for (j = 0; j < i; j++) {
			for (k = i; k < n; k++)
				ut[j][k] -= u[j][i] * ut[i][k];
		}
	}

	return 1;
}


int matrix_inv(float a[][6], int n, float at[][6])
{
	int i, j, k;

	float u[6][6];
	float ut[6][6];

	cholesky(a, n, u);
	invutri(u, n, ut);

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			at[i][j] = 0;
			for (k = 0; k < n; k++) {
				at[i][j] += ut[i][k] * ut[j][k];
			}
		}
	}

	return 1;
}



int matrix_t_mul(float a[][6], float b[][6], float c[][6], int m, int n, int l)
{
	int i, j, k;
	for (i = 0; i < m; i++) {
		for (j = 0; j < l; j++) {
			c[i][j] = 0;
			for (k = 0; k < n; k++) {
				c[i][j] += a[k][i] * b[k][j];
			}
		}
	}

	return 1;
}

int matrix_mul_vector(float a[][6], float* b, float* c, int m, int n)
{
	int i, j;
	for (i = 0; i < m; i++) {
		c[i] = 0;
		for (j = 0; j < n; j++) {
			c[i] += a[i][j] * b[j];
		}
	}

	return 1;
}

int matrix_t_mul_vector(float a[][6], float* b, float* c, int m, int n)
{
	int i, j;
	for (i = 0; i < m; i++) {
		c[i] = 0;
		for (j = 0; j < n; j++) {
			c[i] += a[j][i] * b[j];
		}
	}

	return 1;
}
