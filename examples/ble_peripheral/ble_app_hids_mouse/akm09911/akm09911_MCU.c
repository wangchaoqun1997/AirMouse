/*
* Copyright(C)2014 MediaTek Inc.
* Modification based on code covered by the below mentioned copyright
* and/or permission notice(S).
*/

/* akm09911.c - akm09911 compass driver
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "akm09911.h"

#define DEBUG 0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#define MSE_FUN NRF_LOG_INFO
#define MSE_ERR NRF_LOG_INFO
#define MSE_LOG NRF_LOG_INFO
#define  MAGN_LOG NRF_LOG_INFO
#define  MAG_ERR NRF_LOG_INFO
#define AK09911_ADDRESS AKM09911_I2C_ADDRESS
extern const nrf_drv_twi_t m_twi;
typedef  unsigned char uint8_t;
static int8_t I2C1_Read_Addr8(	const uint8_t slave_addr,const uint8_t read_addr,uint8_t *data,uint8_t data_num)
{
	 char err_code;
		
	  err_code = nrf_drv_twi_tx(&m_twi, slave_addr, &read_addr, 1, false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
		err_code = nrf_drv_twi_rx(&m_twi, slave_addr, data, data_num);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}
static int8_t I2C1_Write_Addr8(	const uint8_t slave_addr,uint8_t write_addr,uint8_t write_value)
{
	  char err_code;
	  uint8_t write[2]={write_addr,write_value};	
	  err_code = nrf_drv_twi_tx(&m_twi, slave_addr, write, sizeof(write), false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}

static int8_t I2C_RxData(uint8_t *data,uint8_t length)
{
	return I2C1_Read_Addr8(AKM09911_I2C_ADDRESS,*data,data,length);
}
static int8_t I2C_TxData(uint8_t *data,uint8_t length)
{
	return I2C1_Write_Addr8(AKM09911_I2C_ADDRESS,*data,*(data+1));
}

static int AKI2C_RxData(char *rxData, int length)
{
	if ((rxData == NULL) || (length < 1))
		return -1;
	return I2C_RxData(rxData,length);
}

static long AKI2C_TxData(char *txData, int length)
{
	if ((txData == NULL) || (length < 2))
		return -1;
	return I2C_TxData(txData,length);
}


/* M-sensor daemon application have set the sng mode */

#define QMCX983_AXIS_X            0
#define QMCX983_AXIS_Y            1
#define QMCX983_AXIS_Z            2

static int cvt_map[3]={2,1,0};
static int cvt_sign[3]={1,-1,1}; //x y z
long AKECS_GetData(int *data)
{


	int  ret;
	static char rbuf0;
	static char rbuf[8]={0};
	rbuf0 = rbuf[0];
	rbuf[0] = AK09911_REG_ST1;
	ret = AKI2C_RxData(rbuf, 1);

	if((rbuf[0] & 0x01) != 0x01){
		nrf_delay_ms(1);
		if((rbuf[0] & 0x01) != 0x01){
			MAG_ERR("AKM8975 data not ready\n");
		//	return -1;
			rbuf[0] = rbuf0;
			goto lastTime;
		}
	}
	nrf_delay_us(20);
	rbuf[0] = AK09911_REG_HXL;
	ret = AKI2C_RxData(&rbuf[0], 8);
	if (ret < 0) {
		MAG_ERR("AKM8975 akm8975_work_func: I2C failed\n");
		return -1;
	}

	//MAGN_LOG("Get device data1: %d, %d, %d, %d !\n",rbuf[0], rbuf[1],rbuf[2],rbuf[3]);
	//MAGN_LOG("Get device data2: %d, %d, %d, %d !\n",rbuf[4], rbuf[5],rbuf[6],rbuf[7]);
lastTime:
	int hw_d[3] = {0};
	int output[3]={0};
	hw_d[0] = (short) (((rbuf[1]) << 8) | rbuf[0]);
	hw_d[1] = (short) (((rbuf[3]) << 8) | rbuf[2]);
	hw_d[2] = (short) (((rbuf[5]) << 8) | rbuf[4]);
	//MSE_LOG(" ----------1--- Hx=%d, Hy=%d, Hz=%d\r\n",hw_d[0],hw_d[1],hw_d[2]);
	
	//MSE_LOG(" ----------2--- Hx=%d, Hy=%d, Hz=%d\r\n",hw_d[0],hw_d[1],hw_d[2]);
	hw_d[0] = hw_d[0] *1.5f ;
	hw_d[1] = hw_d[1] *1.5f ;
	hw_d[2] = hw_d[2] *1.5f ;

	output[cvt_map[0]] = cvt_sign[0]*hw_d[QMCX983_AXIS_X];
	output[cvt_map[1]] = cvt_sign[1]*hw_d[QMCX983_AXIS_Y];
	output[cvt_map[2]] = cvt_sign[2]*hw_d[QMCX983_AXIS_Z];

	data[0] = output[QMCX983_AXIS_X];
	data[1] = output[QMCX983_AXIS_Y];
	data[2] = output[QMCX983_AXIS_Z];

	//MAGN_LOG("AK01988 data [%d, %d, %d ] mg 10mg=1ut_A\n", data[0], data[1], data[2]);

	return 0;

}
long AKECS_SetMode_Con4Measure(void)
{
	char buffer[2];

	/* Set measure mode */
	buffer[0] = AK09911_REG_CNTL2;
	buffer[1] = AK09911_MODE_CON4_MEASURE;

	/* Set data */
	return AKI2C_TxData(buffer, 2);
}
int AKECS_SetMode_PowerDown(void)
{
	char buffer[2];

	/* Set powerdown mode */
	buffer[0] = AK09911_REG_CNTL2;
	buffer[1] = AK09911_MODE_POWERDOWN;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

long AKECS_SetMode(char mode)
{
	long ret;

	switch (mode & 0x1F) {
	case AK09911_MODE_CON4_MEASURE:
	ret = AKECS_SetMode_Con4Measure();
	break;

	case AK09911_MODE_POWERDOWN:
	ret = AKECS_SetMode_PowerDown();
	break;

	default:
//	MAGN_LOG("%s: Unknown mode(%d)", __func__, mode);
	return -1;
	}

	/* wait at least 100us after changing mode */

	nrf_delay_ms(1);
	return ret;
}

long AKECS_Reset()
{
	unsigned char buffer[2];
	long err = 0;
	/* Set measure mode */
		buffer[0] = AK09911_REG_CNTL3;
		buffer[1] = 0x01;
		err = AKI2C_TxData(buffer, 2);
		if (err < 0){
		//MAGN_LOG("%s: Can not set SRST bit.", __func__);
		}
			
		else
			MAGN_LOG("Soft reset is done.");

	/* Device will be accessible 300 us after */

	nrf_delay_ms(1);
	return err;
}

int8_t AKECS_CheckDevice(void)
{
	char buffer[2];
	int ret;

	MAGN_LOG(" AKM check device id");
	/* Set measure mode */
	buffer[0] = AK09911_REG_WIA1;

	/* Read data */
	ret = AKI2C_RxData(buffer, 1);
	MAGN_LOG(" AKM check device id = %x", buffer[0]);
	MAGN_LOG("ret = %d", ret);
	if (ret < 0)
		return ret;

	/* Check read data */
	if (buffer[0] != 0x48)
		return -1;

	return 0;
}
/* Get Msensor Raw data */
static int AKECS_GetRawData(char *rbuf, int size)
{
//	char strbuf[SENSOR_DATA_SIZE];
//	s16 data[3];

//	AKECS_GetData(strbuf, SENSOR_DATA_SIZE);
//	data[0] = (s16)(strbuf[1] | (strbuf[2] << 8));
//	data[1] = (s16)(strbuf[3] | (strbuf[4] << 8));
//	data[2] = (s16)(strbuf[5] | (strbuf[6] << 8));

//	rbuf[0] = data[0];
//	rbuf[1] = data[1];
//	rbuf[2] = data[2];

	return 0;

}

