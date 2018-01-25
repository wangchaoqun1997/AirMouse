//#include "stm32f4xx_hal.h"

//#include "arm_math.h"
//#define BMI160_DEBUG
//#define TIME_DUR_DEBUG
//#define GYRO_CALIBRATIN	
//#define ACC_ZERO_CALIBRATIN
//#define ACC_FILTER 

#include <stdint.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_timer.h"
#include "nrf_delay.h"
typedef unsigned int u32 ;
typedef unsigned char u8 ;
typedef short         s16 ;
typedef char         s8 ;
//APP_TIMER_DEF(sensor_poll_timer_id);
struct bmi160_gyro_t {
	s16 x;/**<gyro X  data*/
	s16 y;/**<gyro Y  data*/
	s16 z;/**<gyro Z  data*/
};
struct bmi160_accel_t {
	s16 x;/**<gyro X  data*/
	s16 y;/**<gyro Y  data*/
	s16 z;/**<gyro Z  data*/
};

#define FACTOR_GYRO_ANDROID   0.0010660980810235f  //  1/938.0f
#define FACTOR_ACC_ANDROID    5.9857177734375e-4f  //  9.807f/16384.0f

struct bmi160_offset_t {
float offsetx;
float offsety;
float offsetz;
};

struct bmi160_offset_t  gyro_offset_tmp={0.0f,0.0f,0.0f};
struct bmi160_offset_t  gyro_offset={-0.000970865309f,-0.00281223701f,0.0080541987f};	//{0.0f,0.0f,0.0f};//

struct bmi160_offset_t  acc_offset_tmp={0.0f,0.0f,0.0f};
struct bmi160_offset_t  acc_offset={-0.39890787f,-0.421067327f,-0.221325114f};


static char firstCall=1;
static char firstACC=1;
struct bmi160_gyro_t  g_gyroxyz;
struct bmi160_accel_t g_accelxyz;

//struct bmi160_accel_t g_last_acc;
float g_last_acc[3];

extern s8    bmi160_read_gyro_xyz(struct bmi160_gyro_t *gyro);
extern s8    bmi160_read_accel_xyz(struct bmi160_accel_t *accel);

extern char  bmi160_get_stat1_data_rdy_intr(u8* v_data_rdy_intr_u8);
extern char  bmi160_get_accel_data_rdy(u8* v_data_rdy_u8);
extern char  bmi160_get_gyro_data_rdy(u8* v_data_rdy_u8);
extern s8 bmi160_get_error_status(u8 *v_fatal_er_u8r,u8 *v_err_code_u8, u8 *v_i2c_fail_err_u8,u8 *v_drop_cmd_err_u8, u8 *v_mag_data_rdy_err_u8);
//extern int32_t ivg_bmg160_init(void);
//extern int32_t ivg_bmg160_data_readout(void);
//extern int32_t ivg_bma2x2_init(void);
//extern int32_t ivg_bma2x2_data(void);
//extern int8_t  ivg_bmi160_init(void);
//extern int8_t  ivg_bmi160_read_data(void);

//extern void  PROCESS_INIT(void);
//extern void  PROCESS_GYRO(float * buf);
//extern void  PROCESS_ACC(float * buf);
//extern void  PROCESS_FUSION(float * buf);

extern void start_during(void );
extern uint32_t get_during_us(void);
char vr_get_data_mode(void);
extern short apply_gyro_offset_X;
extern short apply_gyro_offset_Y;
extern short apply_gyro_offset_Z;
extern short apply_gsensor_offset_X;
extern short apply_gsensor_offset_Y;
extern short apply_gsensor_offset_Z;
int  SENSOR_READ_TEST_2(float * buf)
{ 
	bmi160_read_gyro_xyz(&g_gyroxyz);
	bmi160_read_accel_xyz(&g_accelxyz);
	g_gyroxyz.y -= apply_gyro_offset_X;
	g_gyroxyz.x -= apply_gyro_offset_Y;
	g_gyroxyz.z -= apply_gyro_offset_Z;
	
	g_accelxyz.y -= apply_gsensor_offset_X;
	g_accelxyz.x -= apply_gsensor_offset_Y;
	g_accelxyz.z -= apply_gsensor_offset_Z;
	//gyro for android rad/s
	buf[0] =(float)(g_gyroxyz.y);
	buf[1] =(float)(g_gyroxyz.x);
	buf[2] =(float)(g_gyroxyz.z);

	// acc for android m/s2
	buf[3] =((float)g_accelxyz.y);
	buf[4] =((float)g_accelxyz.x);
	buf[5] =((float)g_accelxyz.z);

}
int  SENSOR_READ_TEST_3(s16 * buf)
{ 
	bmi160_read_gyro_xyz(&g_gyroxyz);
	bmi160_read_accel_xyz(&g_accelxyz);

    //NRF_LOG_INFO("read gyro  x[%d]  y[%d] z[%d] ----- \r\n",g_gyroxyz.y,g_gyroxyz.x,g_gyroxyz.z);
	g_gyroxyz.y -= apply_gyro_offset_X;
	g_gyroxyz.x -= apply_gyro_offset_Y;
	g_gyroxyz.z -= apply_gyro_offset_Z;

	g_accelxyz.y -= apply_gsensor_offset_X;
	g_accelxyz.x -= apply_gsensor_offset_Y;
	g_accelxyz.z -= apply_gsensor_offset_Z;
    //NRF_LOG_INFO("read gyro after x[%d]  y[%d] z[%d] ----- \r\n",g_gyroxyz.y,g_gyroxyz.x,g_gyroxyz.z);
	//gyro for android rad/s
	buf[0] =(g_gyroxyz.y);
	buf[1] =(g_gyroxyz.x);
	buf[2] =(g_gyroxyz.z);

	// acc for android m/s2
	buf[3] =(g_accelxyz.y);
	buf[4] =(g_accelxyz.x);
	buf[5] =(g_accelxyz.z);

}

int  SENSOR_READ_TEST(float * buf)
{ 
#ifdef TIME_DUR_DEBUG	 
	uint32_t duringUs=0;	
	//duringUs=get_during_us();
	start_during();
#endif
	//MPU6050_ONCE(sensor_data);

	bmi160_read_gyro_xyz(&g_gyroxyz);
	bmi160_read_accel_xyz(&g_accelxyz);

    //NRF_LOG_INFO("read gyro  x[%d]  y[%d] z[%d] ----- \r\n",g_gyroxyz.y,g_gyroxyz.x,g_gyroxyz.z);
	g_accelxyz.y -= apply_gsensor_offset_X;
	g_accelxyz.x -= apply_gsensor_offset_Y;
	g_accelxyz.z -= apply_gsensor_offset_Z;

	g_gyroxyz.y -= apply_gyro_offset_X;
	g_gyroxyz.x -= apply_gyro_offset_Y;
	g_gyroxyz.z -= apply_gyro_offset_Z;
    //NRF_LOG_INFO("read gyro after x[%d]  y[%d] z[%d] ----- \r\n",g_gyroxyz.y,g_gyroxyz.x,g_gyroxyz.z);
	//gyro for android rad/s
	buf[0] =(float)(g_gyroxyz.y+3)/938.0f;
	buf[1] =(float)(g_gyroxyz.x+3)/938.0f;
	buf[2] =(float)(g_gyroxyz.z+8)/938.0f;

	// acc for android m/s2
	buf[3] =((float)g_accelxyz.y*9.807f)/16384.0f;
	buf[4] =((float)g_accelxyz.x*9.807f)/16384.0f;
	buf[5] =((float)g_accelxyz.z*9.807f)/16384.0f;


#ifdef TIME_DUR_DEBUG	 
	duringUs=get_during_us();
	printf("us:%d \n\r", duringUs);
#endif
	// printf("%f:%f:%f\n\r",buf[0],buf[1],buf[2]);
	return 0;
}

#define BOARD2_TT
#if 0
void  SENSOR_LAST_PROCESS(float * buf)
{ 
	char mode=0;  

#ifdef TIME_DUR_DEBUG	 
	uint32_t duringUs=0; 
	//duringUs=get_during_us();
	start_during();
#endif 	

#if defined(GYRO_CALIBRATIN)||defined(ACC_ZERO_CALIBRATIN) 	
  static u32 start=0;
#endif	

	//gyro for android rad/s
	mode=vr_get_data_mode();
	if(mode==1)
	{	
		if(firstCall==1)
		{
			buf[0]=0.0f;
			buf[1]=0.0f;
			buf[2]=0.0f;
			firstCall=0;
	    PROCESS_INIT();
			//printf("mode1\n\r");
			bmi160_read_accel_xyz(&g_accelxyz);
			//g_last_acc=g_accelxyz;
			return;
		}
    //printf("mode 1\n\r");
		bmi160_read_gyro_xyz(&g_gyroxyz);
		bmi160_read_accel_xyz(&g_accelxyz);
	

		
	}
	else
	{
		 if(firstCall==1)
		 {
			 firstCall=0;
	     //printf("mode0\n\r");
		 }
	}
	
	
#ifdef BOARD2_TT
	//gyro for android rad/s
	buf[0] =-((float)(g_gyroxyz.x)*(FACTOR_GYRO_ANDROID)  -gyro_offset.offsetx);
	buf[1] =(float)(g_gyroxyz.y)*(-FACTOR_GYRO_ANDROID) -gyro_offset.offsety;
	buf[2] =-((float)(g_gyroxyz.z)*(-FACTOR_GYRO_ANDROID) -gyro_offset.offsetz);

// acc ÂË²¨	
	 // if((buf[0]>0.015f)|(buf[0]<-0.015f)|(buf[1]>0.015f)|(buf[1]<-0.015f)|(buf[2]>0.015f)|(buf[2]<-0.015f))
	//	{
	//		g_accelxyz.x = g_last_acc.x*0.90f + g_accelxyz.x*0.1f;
		//	g_accelxyz.y = g_last_acc.y*0.90f + g_accelxyz.y*0.1f;	
	//		g_accelxyz.z = g_last_acc.z*0.90f + g_accelxyz.z*0.1f;
		//}
	//	else
	//	{

	//	}
		

		
// acc  end	

	
	// acc for android m/s2
	buf[3] =-((float)g_accelxyz.x*(FACTOR_ACC_ANDROID)-acc_offset.offsetx);
	buf[4] =(float)g_accelxyz.y*(-FACTOR_ACC_ANDROID)-acc_offset.offsety;
	buf[5] =-((float)g_accelxyz.z*(-FACTOR_ACC_ANDROID)-acc_offset.offsetz);
#else	
	//gyro for android rad/s
	buf[0] =(float)(g_gyroxyz.x)*(FACTOR_GYRO_ANDROID)  -gyro_offset.offsetx;
	buf[1] =(float)(g_gyroxyz.y)*(-FACTOR_GYRO_ANDROID) -gyro_offset.offsety;
	buf[2] =(float)(g_gyroxyz.z)*(-FACTOR_GYRO_ANDROID) -gyro_offset.offsetz;

// acc ÂË²¨	
	 // if((buf[0]>0.015f)|(buf[0]<-0.015f)|(buf[1]>0.015f)|(buf[1]<-0.015f)|(buf[2]>0.015f)|(buf[2]<-0.015f))
	//	{
	//		g_accelxyz.x = g_last_acc.x*0.90f + g_accelxyz.x*0.1f;
		//	g_accelxyz.y = g_last_acc.y*0.90f + g_accelxyz.y*0.1f;	
	//		g_accelxyz.z = g_last_acc.z*0.90f + g_accelxyz.z*0.1f;
		//}
	//	else
	//	{

	//	}
		

		
// acc  end	

	
	// acc for android m/s2
	buf[3] =(float)g_accelxyz.x*(FACTOR_ACC_ANDROID)-acc_offset.offsetx;;
	buf[4] =(float)g_accelxyz.y*(-FACTOR_ACC_ANDROID)-acc_offset.offsety;
	buf[5] =(float)g_accelxyz.z*(-FACTOR_ACC_ANDROID)-acc_offset.offsetz;
#endif
  #ifndef ACC_ZERO_CALIBRATIN
	if(firstACC==0)
	{
		#ifdef ACC_FILTER
		buf[3] = g_last_acc[0]*0.95f + buf[3]*0.05f;
	  buf[4] = g_last_acc[1]*0.95f + buf[4]*0.05f;	
		buf[5] = g_last_acc[2]*0.95f + buf[5]*0.05f;
		#endif
	}
	firstACC=0;
	g_last_acc[0]=buf[3] ;
	g_last_acc[1]=buf[4] ;
	g_last_acc[2]=buf[5] ;
		
	#endif

	
#ifdef GYRO_CALIBRATIN
	start++;

	if(start>10000)
	{
	}
	else if(start==10000)
	{
		gyro_offset.offsetx=gyro_offset_tmp.offsetx/6000.0f;
		gyro_offset.offsety=gyro_offset_tmp.offsety/6000.0f;
		gyro_offset.offsetz=gyro_offset_tmp.offsetz/6000.0f;	
	}
	else if(start>4000)
	{
		gyro_offset_tmp.offsetx +=buf[0];
		gyro_offset_tmp.offsety +=buf[1];	
		gyro_offset_tmp.offsetz +=buf[2];			
	}
#endif

#ifdef ACC_ZERO_CALIBRATIN
	start++;

	if(start>10000)
	{
		printf("acc calibrate finish\n\r");
	}
	else if(start==10000)
	{
		acc_offset.offsetx=acc_offset_tmp.offsetx/6000.0f;
		acc_offset.offsety=acc_offset_tmp.offsety/6000.0f;
		acc_offset.offsetz=acc_offset_tmp.offsetz/6000.0f;	
	}
	else if(start>4000)
	{
		acc_offset_tmp.offsetx +=buf[3];
		acc_offset_tmp.offsety +=buf[4];	
		acc_offset_tmp.offsetz +=(buf[5]+9.807f);			
	}
#endif

	
	//start_during();
	if(mode==1)
	{
		PROCESS_ACC(&(buf[3]));
		PROCESS_GYRO(buf);	
		PROCESS_FUSION(buf);	
	}
	
#ifdef TIME_DUR_DEBUG	 	
	duringUs=get_during_us();
	printf("us:%d \n\r", duringUs);
#endif

	//printf("%f:%f:%f\n\r",buf[0],buf[1],buf[2]);

}
#endif
 void SENSOR_READ_RAW_INT(float *buf)
{
	
#ifdef BMI160_DEBUG
	u8 v_data_rdy_intr_u8;
	bmi160_get_stat1_data_rdy_intr(&v_data_rdy_intr_u8);
	printf("%d\n\r",v_data_rdy_intr_u8);

	// check witch	ready	
	u8 accrdy;
	u8 gyrordy;
	bmi160_get_accel_data_rdy(&accrdy);
	bmi160_get_gyro_data_rdy(&gyrordy);
	printf("a%d,g%d\n\r",accrdy,gyrordy);
#endif

#ifdef TIME_DUR_DEBUG	
	uint32_t duringUs=0;	
	duringUs=get_during_us();
	start_during();	
#endif	

	bmi160_read_gyro_xyz(&g_gyroxyz);

    //NRF_LOG_INFO("read gyro  x[%d]  y[%d] z[%d] ----- \r\n",g_gyroxyz.y,g_gyroxyz.x,g_gyroxyz.z);
	g_gyroxyz.y -= apply_gyro_offset_X;
	g_gyroxyz.x -= apply_gyro_offset_Y;
	g_gyroxyz.z -= apply_gyro_offset_Z;
    //NRF_LOG_INFO("read gyro after x[%d]  y[%d] z[%d] ----- \r\n",g_gyroxyz.y,g_gyroxyz.x,g_gyroxyz.z);
	//printf("gyx %d\n\r",g_gyroxyz.x);
	buf[0] =(float)(g_gyroxyz.y);
	buf[1] =(float)(g_gyroxyz.x);
	buf[2] =(float)(g_gyroxyz.z);
#ifdef TIME_DUR_DEBUG	
	duringUs=get_during_us();
	printf("us:%d \n\r", duringUs);
#endif	
}


/*TEST USB ONLY*/
int  DUMMY_SENSOR_POLL(float * buf, uint8_t *ms)
{ 
	//gyro for android rad/s
	buf[0] =0.0f;
	buf[1] =0.1f;
	buf[2] =0.2f;

	// acc for android m/s2
	buf[3] =0.3f;
	buf[4] =0.4f;
	buf[5] =0.5f;
	*ms=16;

	return 0;
}/*
float buf[6];
uint8_t interrupter_sensor;
#define SENSOR_POLL_INTERVAL 10
void sensor_poll_start()
{
	app_timer_start(sensor_poll_timer_id,APP_TIMER_TICKS(SENSOR_POLL_INTERVAL),NULL);
}
int tempX,tempY,tempZ;
extern void send_mouse_data(float *buf);
		u8 error_[5]={0};
void sensor_data_poll_handler(void* p_context)
{
	app_timer_stop(sensor_poll_timer_id);
	interrupter_sensor++;	
	//NRF_LOG_INFO("----- interrupter_sensor %d\r\n",interrupter_sensor);
	SENSOR_READ_TEST(buf);
	//bmi160_get_error_status(error_,error_+1,error_+2,error_+3,error_+4);
	send_mouse_data(buf);
	//sensor_poll_start();
	app_timer_start(sensor_poll_timer_id,APP_TIMER_TICKS(SENSOR_POLL_INTERVAL),NULL);
}*/
void SENSOR_INIT(void)
{
	ivg_bmi160_init();
	// ivg_bmg160_init();
	// ivg_bma2x2_init();
	//PROCESS_INIT();
//	app_timer_create(&sensor_poll_timer_id,APP_TIMER_MODE_SINGLE_SHOT,sensor_data_poll_handler);
}




