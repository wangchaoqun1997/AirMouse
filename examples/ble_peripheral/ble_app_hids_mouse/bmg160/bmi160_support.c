/*
****************************************************************************
* Copyright (C) 2016 Bosch Sensortec GmbH
*
* bmi160_support.c
* Date: 2016/03/15
* Revision: 1.0.7 $
*
* Usage: Sensor Driver support file for BMI160 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
#define INCLUDE_BMI160API

#ifdef INCLUDE_BMI160API
//#include "stm32f4xx_hal.h"
#endif

#include "bmi160_support.h"
#include "bmi160.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* Mapping the structure*/
struct bmi160_t s_bmi160;
/* Read the sensor data of accel, gyro and mag*/
struct bmi160_gyro_t gyroxyz;
struct bmi160_accel_t accelxyz;
struct bmi160_mag_xyz_s32_t magxyz;

#ifdef INCLUDE_BMI160API
//extern SPI_HandleTypeDef hspi2;
//extern  s8	ivg_spi_bus_read(u8 * data, u8 cnt, GPIO_TypeDef* cs_port ,uint16_t cs_pin);
//extern  s8  ivg_spi_bus_write(u8 *data, u8 cnt, GPIO_TypeDef* cs_port ,uint16_t cs_pin);
//extern  void ivg_spi_io_init(void);

#endif



void ivg_bmi160_read_acc(struct bmi160_accel_t *accel)
{
	u8 a_data_u8r[6] = {0};
	
	s_bmi160.BMI160_BUS_READ_FUNC(s_bmi160.dev_addr,BMI160_USER_DATA_14_ACCEL_X_LSB__REG,a_data_u8r, 6);

	accel->x = (s16)((((s32)((s8)a_data_u8r[1]))<< 8) | (a_data_u8r[0]));

	accel->y = (s16)((((s32)((s8)a_data_u8r[3]))<< 8) | (a_data_u8r[2]));

	accel->z = (s16)((((s32)((s8)a_data_u8r[5]))<< 8) | (a_data_u8r[4]));

}

BMI160_RETURN_FUNCTION_TYPE ivg_bmi160_init(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE;
 /*	Based on the user need configure I2C or SPI interface.
  *	It is sample code to explain how to use the bmi160 API*/
	#ifdef INCLUDE_BMI160API
	com_rslt = i2c_routine();
	//com_rslt = spi_routine();
	#endif
/*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	company_id
 */
//	while(1){
	   //HAL_Delay(10);
	   	com_rslt += bmi160_init(&s_bmi160);
		//NRF_LOG_INFO("bmi160 1chip_id=%d\n\r",s_bmi160.chip_id);
	   	com_rslt += bmi160_init(&s_bmi160);
		//NRF_LOG_INFO("bmi160 2chip_id=%d\n\r",s_bmi160.chip_id);
		//HAL_Delay(500);
	//}
	s_bmi160.delay_msec(100);
// use APPLICATION_HEAD_TRACKING mode 
								
	// APPLICATION_HEAD_TRACKING:
	/*Set the accel mode as Normal write in the register 0x7E*/
	com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
	s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
	s_bmi160.delay_msec(100);
	/*Set the gyro mode as Normal write in the register 0x7E*/
	com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
	s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
	s_bmi160.delay_msec(100);
	/* Set the accel bandwidth as Normal */
	com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
	s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	s_bmi160.delay_msec(100);
	/* Set the gryo bandwidth as Normal */
	com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	/* set gyro data rate as 1600Hz*/
	com_rslt += bmi160_set_gyro_output_data_rate(BMI160_GYRO_OUTPUT_DATA_RATE_400HZ);
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	/* set accel data rate as 1600Hz*/
	com_rslt += bmi160_set_accel_output_data_rate(BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ, BMI160_ACCEL_OSR4_AVG1);
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/

  // range 
 //  bmi160_set_gyro_range not do , just as defaut 2000
 //  bmi160_set_acc_range   not do , just as defaut 2G
 
 #if 0
// interrupt 
	/* Configure the in/out control of interrupt1*/
	com_rslt = bmi160_set_output_enable(0,BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	s_bmi160.delay_msec(100);
	
	/* Configure the in/out control of interrupt2*/
	com_rslt += bmi160_set_output_enable(1,BMI160_ENABLE);//BMI160_DISABLE
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
	/* Configure the interrupt1 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(0,BMI160_LEVEL_HIGH);
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the interrupt2 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(1,BMI160_LEVEL_HIGH);
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);


    /*interrupt edge :use default is level*/
	//bmi160_set_intr_edge_ctrl()

    // map chanel 1 data rdy 
	com_rslt += bmi160_set_intr_data_rdy(BMI160_INTR1_MAP_DATA_RDY, BMI160_ENABLE);
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

   //enable rdy interrupt
	com_rslt += bmi160_set_intr_enable_1(BMI160_DATA_RDY_ENABLE,BMI160_ENABLE);
	s_bmi160.delay_msec(100);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

   // set interrupt latch   : nolatch default
    //bmi160_set_latch_intr()

	// bmi160_get_stat1_data_rdy_intr(u8*v_data_rdy_intr_u8)
#endif
	
	return com_rslt;


}

BMI160_RETURN_FUNCTION_TYPE ivg_bmi160_read_data(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	/* read gyro data*/
	com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
	/* read accel data */
	com_rslt += bmi160_read_accel_xyz(&accelxyz);
	/* read mag data */
	//com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);
	//printf("acc =%d ; %d ; %d\n\r",accelxyz.x,accelxyz.y,accelxyz.z);
	//printf("gy  =%d ; %d ; %d\n\r",gyroxyz.x,gyroxyz.y,gyroxyz.z);
  return 0;
}
/*!
 *	@brief This function used for initialize the sensor
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_initialize_sensor(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE;
 /*	Based on the user need configure I2C or SPI interface.
  *	It is sample code to explain how to use the bmi160 API*/
	#ifdef INCLUDE_BMI160API
	com_rslt = i2c_routine();
	//com_rslt = spi_routine(); 
	#endif
/*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	company_id
 */
	com_rslt += bmi160_init(&s_bmi160);
	/**** standard 9Dof with FIFO output****/
	com_rslt += bmi160_config_running_mode(STANDARD_UI_9DOF_FIFO);
	return com_rslt;
}

/*!
 *	@brief This Function used to read the sensor data using
 *	different running mode
 *	@param v_running_mode_u8 : The value of running mode
 *      Description                |  value
 * --------------------------------|----------
 *  STANDARD_UI_9DOF_FIFO          |   0
 *	STANDARD_UI_IMU_FIFO           |   1
 *	STANDARD_UI_IMU                |   2
 *	STANDARD_UI_ADVANCEPOWERSAVE   |   3
 *	ACCEL_PEDOMETER                |   4
 *	APPLICATION_HEAD_TRACKING      |   5
 *	APPLICATION_NAVIGATION         |   6
 *	APPLICATION_REMOTE_CONTROL     |   7
 *	APPLICATION_INDOOR_NAVIGATION  |   8
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_config_running_mode(
u8 v_running_mode_u8)
{
	struct gyro_sleep_setting gyr_setting;
	struct bmi160_fifo_data_header_t header_data;

	/* Variable used for get the status of mag interface*/
	u8 v_mag_interface_u8 = BMI160_INIT_VALUE;
	u8 v_bmm_chip_id_u8 = BMI160_INIT_VALUE;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;
		/* Configure the gyro sleep setting based on your need*/
	if (v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE) {
		gyr_setting. sleep_trigger = BMI160_SLEEP_TRIGGER;
		gyr_setting. wakeup_trigger = BMI160_WAKEUP_TRIGGER;
		gyr_setting. sleep_state = BMI160_SLEEP_STATE;
		gyr_setting. wakeup_int = BMI160_WAKEUP_INTR;
	}
	/* The below code used for enable and
	disable the secondary mag interface*/
	com_rslt = bmi160_get_if_mode(&v_mag_interface_u8);
	if (((v_running_mode_u8 == STANDARD_UI_IMU_FIFO) ||
	(v_running_mode_u8 == STANDARD_UI_IMU) ||
	(v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE) ||
	(v_running_mode_u8 == APPLICATION_NAVIGATION) ||
	(v_running_mode_u8 == ACCEL_PEDOMETER) ||
	(v_running_mode_u8 == APPLICATION_REMOTE_CONTROL) ||
	(v_running_mode_u8 == APPLICATION_INDOOR_NAVIGATION))
	&& (v_mag_interface_u8 == BMI160_MAG_INTERFACE_ON_PRIMARY_ON)) {
		com_rslt +=
		bmi160_set_bmm150_mag_and_secondary_if_power_mode(
		MAG_SUSPEND_MODE);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		com_rslt += bmi160_set_if_mode(
		BMI160_MAG_INTERFACE_OFF_PRIMARY_ON);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	}
	if (((v_running_mode_u8 == STANDARD_UI_9DOF_FIFO)
		|| (v_running_mode_u8 == APPLICATION_HEAD_TRACKING) ||
		(v_running_mode_u8 == APPLICATION_NAVIGATION)) &&
		(v_mag_interface_u8 == BMI160_MAG_INTERFACE_OFF_PRIMARY_ON)) {
			/* Init the magnetometer */
			com_rslt += bmi160_bmm150_mag_interface_init(
			&v_bmm_chip_id_u8);
			/* bmi160_delay_ms in ms*/
			s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	switch (v_running_mode_u8) {
	case STANDARD_UI_9DOF_FIFO:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */ 
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
		BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
		BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ, BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/***** read FIFO data based on interrupt*****/
		com_rslt += bmi160_interrupt_configuration();
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO mag*/
		com_rslt += bmi160_set_fifo_mag_enable(FIFO_MAG_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		/* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt1*/
		/* Enable FIFO water mark interrupts in INT_EN[1] */
		com_rslt += bmi160_set_intr_enable_1(BMI160_FIFO_WM_ENABLE,
		BMI160_ENABLE);
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_INIT_VALUE,
		FIFO_WM_INTERRUPT_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_ENABLE,
		FIFO_WM_INTERRUPT_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the fifo water mark*/
		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read the FIFO data*/
		com_rslt +=  bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
		&header_data);
	break;
	case STANDARD_UI_IMU_FIFO:
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/***** read FIFO data based on interrupt*****/
		com_rslt += bmi160_interrupt_configuration();
		/* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable FIFO water mark interrupts in INT_EN[1] */
		com_rslt += bmi160_set_intr_enable_1(BMI160_FIFO_WM_ENABLE,
		BMI160_ENABLE);
		/* Enable the FIFO water mark interrupt1*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_INIT_VALUE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the fifo water mark as 10*/
		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);
		/* read the FIFO data*/
		com_rslt +=  bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
		&header_data);
	break;
	case STANDARD_UI_IMU:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case STANDARD_UI_ADVANCEPOWERSAVE:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/

		/* Enable any motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_X_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable any motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_Y_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable any motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_Z_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_X_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_Y_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_Z_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro sleep trigger*/
		com_rslt += bmi160_set_gyro_sleep_trigger(
		gyr_setting.sleep_trigger);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro wakeup trigger*/
		com_rslt += bmi160_set_gyro_wakeup_trigger(
		gyr_setting.wakeup_trigger);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro sleep state*/
		com_rslt += bmi160_set_gyro_sleep_state(
		gyr_setting.sleep_state);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro wakeup interrupt*/
		com_rslt += bmi160_set_gyro_wakeup_intr(gyr_setting.wakeup_int);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case ACCEL_PEDOMETER:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_LOWPOWER);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as SUSPEND write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_SUSPEND);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as OSR4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 25Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ,
			BMI160_ACCEL_OSR4_AVG1);
		/* 10 not available*/
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case APPLICATION_HEAD_TRACKING:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 1600Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
		BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 1600Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
		BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ, BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		/* read mag data */
		com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);
	break;
	case APPLICATION_NAVIGATION:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		/* read mag data*/
		com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);
	break;
	case APPLICATION_REMOTE_CONTROL:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data */
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case APPLICATION_INDOOR_NAVIGATION:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_400HZ);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		break;
	}

	return com_rslt;

}
/*!
 *	@brief This function used for interrupt configuration
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_interrupt_configuration(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	/* Configure the in/out control of interrupt1*/
	com_rslt = bmi160_set_output_enable(BMI160_INIT_VALUE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the in/out control of interrupt2*/
	com_rslt += bmi160_set_output_enable(BMI160_ENABLE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the interrupt1 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_INIT_VALUE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the interrupt2 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_ENABLE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	return com_rslt;
}
#ifdef INCLUDE_BMI160API
#define MASK_DATA1	0xFF
#define MASK_DATA2	0x80
#define MASK_DATA3	0x7F
#define SPI_BUFFER_LEN 13
#define I2C_BUFFER_LEN 13
/*!
 *	@brief Used for I2C initialization
 *	@note
 *	The following function is used to map the
 *	I2C bus read, write, bmi160_delay_ms and
 *	device address with global structure bmi160_t
*/
#include "nrf_drv_twi.h"
extern const nrf_drv_twi_t m_twi1;
typedef  unsigned char uint8_t;
static int8_t I2C1_Read_Addr8(	const uint8_t slave_addr,const uint8_t read_addr,uint8_t *data,uint8_t data_num)
{
	 char err_code;
		
	  err_code = nrf_drv_twi_tx(&m_twi1, slave_addr, &read_addr, 1, false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
		err_code = nrf_drv_twi_rx(&m_twi1, slave_addr, data, data_num);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}
static int8_t I2C2_Write_Addr8(	const uint8_t slave_addr,uint8_t write_addr,uint8_t write_value)
{
	  char err_code;
	  uint8_t write[2]={write_addr,write_value};	
	  err_code = nrf_drv_twi_tx(&m_twi1, slave_addr, write, sizeof(write), false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}
s8 i2c_routine(void)
{
/*--------------------------------------------------------------------------*
 *  By using bmi160 the following structure parameter can be accessed
 *	Bus write function pointer: BMI160_WR_FUNC_PTR
 *	Bus read function pointer: BMI160_RD_FUNC_PTR
 *	bmi160_delay_ms function pointer: bmi160_delay_ms_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	s_bmi160.bus_write = bmi160_i2c_bus_write;
	s_bmi160.bus_read = bmi160_i2c_bus_read;
	s_bmi160.delay_msec = bmi160_delay_ms;
	s_bmi160.dev_addr = BMI160_I2C_ADDR1;

	return BMI160_INIT_VALUE;
}
/*!
 *	@brief Used for SPI initialization
 *	@note
 *	The following function is used to map the
 *	SPI bus read, write and bmi160_delay_ms
 *	with global structure bmi160
*/
s8 spi_routine(void)
{
/*--------------------------------------------------------------------------*
 *  By using bmi160 the following structure parameter can be accessed
 *	Bus write function pointer: BMI160_WR_FUNC_PTR
 *	Bus read function pointer: BMI160_RD_FUNC_PTR
 *	bmi160_delay_ms function pointer: bmi160_delay_ms_msec
 *--------------------------------------------------------------------------*/

	s_bmi160.bus_write = bmi160_spi_bus_write;
	s_bmi160.bus_read = bmi160_spi_bus_read;
	s_bmi160.delay_msec = bmi160_delay_ms;
	//ivg_spi_io_init();
	return BMI160_INIT_VALUE;
}
/**************************************************************/
/**\name I2C/SPI read write function */
/**************************************************************/
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	Configure the below code to your SPI or I2C driver
*
*-----------------------------------------------------------------------*/
 /*!
 *	@brief : The function is used as I2C bus read
 *	@return : Status of the I2C read
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register, will data is going to be read
 *	@param reg_data : This data read from the sensor, which is hold in an array
 *	@param cnt : The no of byte of data to be read
 */
s8 bmi160_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	#ifdef INCLUDE_BMI160API
#if 0
	u8 array[I2C_BUFFER_LEN] = {BMI160_INIT_VALUE};
	u8 stringpos = BMI160_INIT_VALUE;

	array[BMI160_INIT_VALUE] = reg_addr;
	/* Please take the below function as your reference
	 * for read the data using I2C communication
	 * add your I2C rad function here.
	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
     * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	for (stringpos = BMI160_INIT_VALUE; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
#else
	I2C1_Read_Addr8(dev_addr,reg_addr,reg_data,cnt);
#endif
	#endif
	return (s8)ierror;

}
 /*!
 *	@brief : The function is used as I2C bus write
 *	@return : Status of the I2C write
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be written
 *	@param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	@param cnt : The no of byte of data to be write
 */
s8 bmi160_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	#ifdef INCLUDE_BMI160API
#if 0
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMI160_INIT_VALUE;

	array[0] = reg_addr;
	for (stringpos = BMI160_INIT_VALUE; stringpos< cnt; stringpos++)
		array[stringpos + BMI160_GEN_READ_WRITE_DATA_LENGTH]= *(reg_data + stringpos);
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
#else
	//I2C1_Write_Addr8(dev_addr,reg_addr,reg_data,cnt);
	I2C2_Write_Addr8(dev_addr,reg_addr,*reg_data);
#endif
	#endif
	return (s8)ierror;
}
/*!
 *	@brief : The function is used as SPI bus read
 *	@return : Status of the SPI read
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be read
 *	@param reg_data : This data read from the sensor,
 *	which is hold in an array
 *	@param cnt : The no of byte of data to be read
 */
s8 bmi160_spi_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	#ifdef INCLUDE_BMI160API

	u8 array[SPI_BUFFER_LEN] = {MASK_DATA1};
	u8 stringpos;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as 0)*/
	array[BMI160_INIT_VALUE] = reg_addr|MASK_DATA2;
	/*
	* Please take the below function as your reference for
	* read the data using SPI communication
	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
	* add your SPI read function here
	* ierror is an return value of SPI read function
	* Please select your valid return value
	* In the driver SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done in the SPI read
	* and write string function
	* For more information please refer data sheet SPI communication:
	*/
	//ivg_spi_bus_read(array,  cnt, GPIOB ,GPIO_BMI160_CS_Pin);

	for (stringpos = BMI160_INIT_VALUE; stringpos
	< cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos
		+ BMI160_GEN_READ_WRITE_DATA_LENGTH];
	#endif
	return (s8)ierror;
}
/*!
 *	@brief : The function is used as SPI bus write
 *	@return : Status of the SPI write
 *	@param dev_addr : The device address of the sensor
 *	@param reg_addr : Address of the first register,
 *	will data is going to be written
 *	@param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	@param cnt : The no of byte of data to be write
 */
s8 bmi160_spi_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	#ifdef INCLUDE_BMI160API

	u8 array[SPI_BUFFER_LEN * C_BMI160_BYTE_COUNT];
	u8 stringpos = BMI160_INIT_VALUE;

	for (stringpos = BMI160_INIT_VALUE;
	stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done:
		because it ensure the
		   0 and 1 of the given value
		   It is done only for 8bit operation*/
		array[stringpos * C_BMI160_BYTE_COUNT] =
		(reg_addr++) & MASK_DATA3;
		array[stringpos * C_BMI160_BYTE_COUNT +
		BMI160_GEN_READ_WRITE_DATA_LENGTH] =
		*(reg_data + stringpos);
	}
		/* Please take the below function as your reference
	 * for write the data using SPI communication
	 * add your SPI write function here.
	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*2)"
	 * ierror is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
     
 	 //ivg_spi_bus_write(array,  (cnt*C_BMI160_BYTE_COUNT), GPIOB ,GPIO_BMI160_CS_Pin);

		
	 #endif
	return (s8)ierror;
}
#endif
/*!
 *	@brief This function is an example for delay
 *	@param msek: delay in milli seconds
 *	@return : communication result
 */
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "softdevice_handler_appsh.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "app_button.h"
#include "ble_advertising.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_delay.h"
void bmi160_delay_ms(u32 msek)
{
 /* user delay*/
#ifdef INCLUDE_BMI160API
	//HAL_Delay(msek);
	nrf_delay_ms(msek);
#endif
}

BMI160_RETURN_FUNCTION_TYPE bmi160_suspend(void){
	bmi160_set_command_register(ACCEL_SUSPEND);
		/* bmi160_delay_ms in ms*/
	s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
	bmi160_set_command_register(GYRO_MODE_SUSPEND);
		/* bmi160_delay_ms in ms*/
	s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
}
BMI160_RETURN_FUNCTION_TYPE bmi160_resume(void){
	bmi160_set_command_register(ACCEL_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
	s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
	/*Set the gyro mode as Normal write in the register 0x7E*/
	bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
	s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
}
