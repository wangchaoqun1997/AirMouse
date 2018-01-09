#include "mms400_update.h"
#include "mms400_fw.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#if      (TOUCH_IC_SELECT == TOUCHPAD_MELFAS_MMS427)
u8  FW_CHIP_CODE[4] = {0x4D,0x34,0x48,0x32};     //"M4H2"
#define CHIP_NAME           "MMS427"
//#define CHIP_MODEL		CHIP_MMS427
#endif
#if     (TOUCH_IC_SELECT == TOUCHPAD_MELFAS_MMS438)
u8  FW_CHIP_CODE[4] = {0x4D,0x34,0x48,0x30}; 	//"M4H0"
#define CHIP_NAME           "MMS438"
//#define CHIP_MODEL		CHIP_MMS438
#endif

struct firmware melfas_fw = 
{
	sizeof(MELFAS_MFSB),
       &MELFAS_MFSB[0]
};

static int mms_isc_read_status(u8 addr)
{
	u8 write_buf[6] =  ISC_CMD_READ_STATUS;
	u8 read_buf[4];
	u8 result = 0;
	int cnt = 100;
	int ret = 0;
	
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[START]mms_isc_read_status\r\n");	
	#endif
	do {
		if(mms_i2c_read(addr, write_buf, 6, read_buf, 1)){
			USART1_printf("[ERROR] mms_i2c_read failed\r\n");			
			ret = -1;
			goto ERROR;
		}
		result = read_buf[0];
		
		if(result == ISC_STATUS_DONE){
			ret = 0;
			break;
		}
		else if(result == ISC_STATUS_BUSY){
			ret = -1;
			delay_ms(1);
		}
		else{
			USART1_printf(" [ERROR] mms_isc_read_status wrong value [0x%x ]\r\n", result);
			ret = -1;
			delay_ms(1);
		}	
	} while (--cnt);

	if (!cnt) {
		USART1_printf( " [ERROR]mms_isc_read_status  count overflow - cnt [%d] status [0x%x ]\r\n", cnt, result);
		goto ERROR;
	}
	
#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[DONE]mms_isc_read_status successful\r\n");	
#endif
	
	return ret;

ERROR:
	return ret;	
}

static int mms_isc_erase_page(u8 addr)
{
		u8 write_buf[6] =ISC_CMD_ERASE_ALL;//ISC_CMD_ERASE_PAGE;
		
#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[START] mms isc erase page\r\n");	
#endif

	if(mms_i2c_write(addr, write_buf, 6)){
		USART1_printf(" [ERROR] mms_i2c_write failed\r\n");
		goto ERROR;
	}
	
	if(mms_isc_read_status(addr) != 0){
		goto ERROR;
	}
	
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf(" [DONE] mms_isc_erase_page - Offset\r\n");
	#endif
		
	return 0;

ERROR:
	return -1;
}

static int mms_isc_program_page(u8 addr, int offset,const u8 *data, int length)
{
	u8 write_buf[134] = ISC_CMD_WRITE_PAGE;//ISC_CMD_PROGRAM_PAGE;
	int i;

	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[START]mms_isc_program_page\r\n");	
	#endif

	if( length > ISC_PAGE_SIZE ){
		USART1_printf("[ERROR] page length overflow\r\n");
		goto ERROR;
	}
		
	write_buf[4] = (u8)(((offset)>>8)&0xFF );
	write_buf[5] = (u8)(((offset)>>0)&0xFF );
	
	for(i = 0; i < length; i++)
	{
		write_buf[i+6] = *data;
		data++;
	}

	if(mms_i2c_write(addr,write_buf,length + 6))
	{
		USART1_printf("[ERROR] mms_i2c_write failed\r\n");	
		goto ERROR;
	}
	
	if(mms_isc_read_status(addr) != 0){
		goto ERROR;
	}
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[DONE]mms_isc_program_page - Offset[0x%x ] Length[%d ]\r\n", offset, length);	
	#endif
	return 0;

ERROR:
	return 1;
}

static int mms_isc_read_page(u8 addr,int offset, u8 *data)
{
	u8 write_buf[6] =ISC_CMD_READ_PAGE;

	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[START] mms_isc_read_page\r\n");
	#endif

	write_buf[4] = (u8)(((offset)>>8)&0xFF );
	write_buf[5] = (u8)(((offset)>>0)&0xFF );

	if(mms_i2c_read(addr, write_buf, 6, data, ISC_PAGE_SIZE)){
		USART1_printf("[ERROR] mms_i2c_read\r\n");
		goto ERROR;
	}
	
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[DONE] mms_isc_read_page- Offset [0x%x ]\r\n", offset);	
	#endif
		
	return 0;

ERROR:
	return 1;
}

static int mms_isc_exit(u8 addr)
{
	u8 write_buf[6] = ISC_CMD_EXIT;
	
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[START]mms_isc_exit\r\n");	
	#endif

	if(mms_i2c_write(addr, write_buf, 6)){
		USART1_printf("[ERROR] mms_isc_exit-mms_i2c_write failed\r\n");	
		goto ERROR;
	}
	
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("[DONE]mms_isc_exit\r\n");	
	#endif
	
	return 0;

ERROR:
	return -1;
}

int mms_flash_fw(u8 Addr,  const u8 *fw_data, u16 fw_size, bool force, bool section)
{
	struct mip_bin_tail *bin_info;

	int t;
	int retry = 3;
	int nRet = 0;
	u8 rbuf[ISC_PAGE_SIZE];
	int offset_start = 0;
	int array_offset = 0;
	int bin_size = 0;
	int offset = 0;

	
	u16 ver_chip[MMS_FW_MAX_SECT_NUM];

	u16 tail_size = 0;
	u8 tail_mark[4] = MIP_BIN_TAIL_MARK;

	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("%s[START]\r\n",__func__);	
	#endif
	
	tail_size = (fw_data[fw_size - 5] << 8) | fw_data[fw_size - 6];
	if (tail_size != MIP_BIN_TAIL_SIZE) {
		USART1_printf("%s [ERROR] wrong tail size [%d]\r\n", __func__, tail_size);
		nRet = fw_err_file_type;
		goto ERROR_FILE;
	}	

	//Check bin format
	for(t = 0; t < 4; t++)
	{
		if(fw_data[fw_size - tail_size +t] != tail_mark[t] )
		{
			USART1_printf("%s [ERROR] wrong tail mark\r\n", __func__);
			nRet = fw_err_file_type;
			goto ERROR_FILE;
		}
	}
	//Read bin info
	bin_info = (struct mip_bin_tail *)&fw_data[fw_size - tail_size];
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("%s - bin_info : bin_len[%d] hw_cat[0x%x ] date[%x ] time[%x ] tail_size[%d]\r\n", __func__, bin_info->bin_length, bin_info->hw_category, bin_info->build_date, bin_info->build_time, bin_info->tail_size);
	#endif

	//Check chip code
	for(t =0;t<4;t++)
	{
		if(bin_info->chip_name[t] != FW_CHIP_CODE[t])
		{
			USART1_printf("%s [ERROR] F/W file is not for %s\r\n", __func__, CHIP_NAME);
			nRet = fw_err_file_type;
		       goto ERROR_FILE;
		}
	}
	
	//Check F/W version
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("%s - F/W file version [0x%x 0x%x 0x%x 0x%x ]\r\n", __func__, bin_info->ver_boot, bin_info->ver_core, bin_info->ver_app, bin_info->ver_param);
	#endif
	if (force == true) 
	{
		//Force update
		USART1_printf("%s - Skip chip firmware version check\r\n", __func__);
	} 
	else 
	{
	//Read firmware version from chip
		while (retry--) 
		{
			if (!MMS_Get_FW_Version_u16(TOUCH_DEVICE_ADDR, ver_chip)) 
			{
				break;
			} 
			else 
			{
				MMS_Reboot();
			}
		}

		if (retry < 0) 
		{
			USART1_printf("%s [ERROR] Unknown chip firmware version\r\n", __func__);
		} 
		else 
		{
                     #if     (UPDATE_LOG_ENABLE == 1)
			USART1_printf("%s - Chip firmware version [0x%x 0x%x 0x%x 0x%x ]\r\n", __func__, ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);
			#endif

			if ((ver_chip[0] == bin_info->ver_boot) && (ver_chip[1] == bin_info->ver_core) && (ver_chip[2] == bin_info->ver_app) && (ver_chip[3] == bin_info->ver_param)) 
			{
				USART1_printf("%s - Chip firmware is already up-to-date\r\n", __func__);
				nRet = fw_err_uptodate;
				goto UPTODATE;
			}
		}
	}

	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("%s - Start offset[0x%x ]\r\n", __func__, offset_start);
	#endif

	//Read bin data
	bin_size = bin_info->bin_length;

	//Erase
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("%s - Erase\r\n", __func__);
	#endif
	nRet = mms_isc_erase_page(Addr);
	if (nRet != 0) {
		USART1_printf("%s [ERROR] mip_isc_erase_mass\r\n", __func__);
		nRet = fw_err_download;
		goto ERROR_UPDATE;
	}
	
       //Download & Verify
       #if     (UPDATE_LOG_ENABLE == 1)
       USART1_printf("%s - Download & Verify\r\n", __func__);
	#endif
	offset = bin_size -  ISC_PAGE_SIZE;
	offset_start = 0;
	array_offset = fw_data[ISC_PAGE_SIZE /8];
	t = 0;
	while (offset >= offset_start)
	{
		//Write page

		if (mms_isc_program_page(Addr, offset, &fw_data[offset+array_offset], ISC_PAGE_SIZE))
		{
			USART1_printf("%s [ERROR] mip_isc_write_page : offset[0x%x ]\r\n", __func__, offset);
			nRet = fw_err_download;
			goto ERROR_UPDATE;
		}
		#if     (UPDATE_LOG_ENABLE == 1)
		USART1_printf("%s - mip_isc_write_page : offset[0x%x ]\r\n", __func__, offset);
		#endif

		//Verify page
		if (mms_isc_read_page(Addr, offset, rbuf)) {
			USART1_printf("%s [ERROR] mip_isc_read_page : offset[0x%x ]\r\n", __func__, offset);
			nRet = fw_err_download;
			goto ERROR_UPDATE;
		}
		#if     (UPDATE_LOG_ENABLE == 1)
		USART1_printf("%s - mip_isc_read_page : offset[0x%x ]\r\n", __func__, offset);
		#endif


              for(t =0; t < ISC_PAGE_SIZE; t++)
              {
              	if(rbuf[t] != fw_data[offset + t+ array_offset])
              	{
	              	USART1_printf("%s [ERROR] Verify failed : offset[0x%x ] SN in the page[%d]\r\n", __func__, offset,t);
				nRet = fw_err_download;
				goto ERROR_UPDATE;
              	
              	}
              }

		//Next offset
		offset -= ISC_PAGE_SIZE;
	}

	//Exit ISC mode
	#if     (UPDATE_LOG_ENABLE == 1)
	USART1_printf("%s - Exit\r\n", __func__);
	#endif
       mms_isc_exit(Addr);
	
	//Reset chip
	MMS_Reboot();
	
	//Check chip firmware version
	if (MMS_Get_FW_Version_u16(Addr, ver_chip)) {
		USART1_printf("%s [ERROR] Unknown chip firmware version\r\n", __func__);
		nRet = fw_err_download;
		goto ERROR_UPDATE;
	} else {
		if ((ver_chip[0] == bin_info->ver_boot) && (ver_chip[1] == bin_info->ver_core) && (ver_chip[2] == bin_info->ver_app) && (ver_chip[3] == bin_info->ver_param)) {
			#if     (UPDATE_LOG_ENABLE == 1)
			USART1_printf("%s - Version check OK\r\n", __func__);
			#endif
			nRet = 0;
			goto EXIT;
		} else {
			USART1_printf("%s [ERROR] Version mismatch after flash. Chip[0x%x 0x%x 0x%x 0x%x ] File[0x%x 0x%x  0x%x  0x%x ]\r\n", __func__, ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3], bin_info->ver_boot, bin_info->ver_core, bin_info->ver_app, bin_info->ver_param);
			nRet = fw_err_download;
			goto ERROR_UPDATE;
		}
	}

	
UPTODATE:
	USART1_printf("%s [DONE]\r\n", __func__);
	goto EXIT;
ERROR_UPDATE:
	USART1_printf("%s [ERROR_UPDATE]\r\n", __func__);
	return nRet;
	
ERROR_FILE:
	USART1_printf("%s [ERROR_FILE]\r\n", __func__);
	return nRet;
	
EXIT:
	MMS_Reboot();
	return nRet;
}

