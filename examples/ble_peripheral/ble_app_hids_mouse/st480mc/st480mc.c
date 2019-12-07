/* 
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



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_twi.h"
#define MSE_FUN NRF_LOG_INFO
#define MSE_ERR NRF_LOG_INFO
#define MSE_LOG NRF_LOG_INFO

#define ST480MC_ADDR				0x0C
#define ST480MC_BURST_MODE	0x1E
#define ST480MC_SINGLR_MODE	0x3E
#define ST480MC_READ_DATA		0x4E
#define ST480MC_WRITE_DATA	0x60
#define ST480MC_SENS_XY			667		// X,Y sensitivity; unit: LSB/Gauss
#define ST480MC_SENS_Z			400		// Z sensitivity;		unit: LSB/Gauss

extern const nrf_drv_twi_t m_twi;

int8_t I2C1_Read_Addr8(	const uint8_t slave_addr,const uint8_t read_addr,uint8_t *data,uint8_t data_num)
{
	 char err_code; 
	  err_code = nrf_drv_twi_tx(&m_twi, slave_addr, &read_addr, 1, false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
			//MSE_ERR("Transfer OK1 :0x%x\r\n",slave_addr);
    }else{
			MSE_ERR("Transfer Error1\r\n");
		}
	

		err_code = nrf_drv_twi_rx(&m_twi, slave_addr, data, data_num);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
			//MSE_ERR("Transfer OK2\r\n");
    }else{
			MSE_ERR("Transfer Error2\r\n");
		}
	  return (int8_t)err_code;
}
int8_t I2C1_Write_Addr8(	const uint8_t slave_addr,uint8_t write_addr,uint8_t write_value)
{
	  char err_code;
	  uint8_t write[2]={write_addr,write_value};	
	  err_code = nrf_drv_twi_tx(&m_twi, slave_addr, write, sizeof(write), false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}
 



void I2C_Read_NBytes(unsigned char deviceAddr, unsigned char regAddr, unsigned char readLen, unsigned char *readBuf){
	
I2C1_Read_Addr8(deviceAddr,regAddr,readBuf,readLen);
}

 
void I2C_Write_NBytes(unsigned char deviceAddr, unsigned char regAddr, unsigned char writeVal){
	I2C1_Write_Addr8(deviceAddr,regAddr,writeVal);
}

 

/******************************************************************
* SH480MC initialization function
* Note: Single measurement mode
******************************************************************/
int initST480MC(void) 
{
	unsigned char tempdata[1] = {0};// Set sinlge measurement mode	
	I2C_Read_NBytes(ST480MC_ADDR, ST480MC_SINGLR_MODE, 1, tempdata);
	MSE_ERR("initST480MC Finish\r\n");
}	


/******************************************************************
* Read ST480MC X,Y,Z data
* Note: ODR=65Hz
******************************************************************/
bool isST480MCReadOK=false;
void readST480MC( int *data )
{
	unsigned char magData[7]={0,0,0,0,0,0,0};
	short int magX=0, magY=0, magZ=0;
//float magXFData=0.0f, magYFData=0.0f, magZFData=0.0f;	// Read Status, magX(XH,XL), magY(YH,YL), magZ(ZH,ZL);
	I2C_Read_NBytes(ST480MC_ADDR, ST480MC_READ_DATA, 7, magData);// magData[0]:Status register value;
	if((magData[0] & 0x10) == 0)
	{
		magX = (short int)(magData[1] << 8) | magData[2];
		magY = (short int)(magData[3] << 8) | magData[4];
		magZ = (short int)(magData[5] << 8) | magData[6];
// convert to Gauss
//magXFData = magX/ST480MC_SENS_XY*1.0f;
//magYFData = magY/ST480MC_SENS_XY*1.0f;
//magZFData = magZ/ST480MC_SENS_Z*1.0f;
//printf("%d %d %d\n", magX, magY, magZ);
		

		//MSE_ERR("readST480MC : %d %d %d \r\n",data[0], data[1], data[2]);
		//MSE_ERR("readST480MC : %d %d %d \r\n",magX, magY, magZ);
		isST480MCReadOK = true;
	}else{
	isST480MCReadOK = false;
		//MSE_ERR("readST480MC : Fail !!!!!! %d \r\n",magData[0]);
	}		
		data[0] = - (int)magY;
		data[1] = - (int)magX;
		data[2] = - (int)magZ;
// Set sinlge mode, start next measurement;
	I2C_Read_NBytes(ST480MC_ADDR, ST480MC_SINGLR_MODE, 1, magData);
}
