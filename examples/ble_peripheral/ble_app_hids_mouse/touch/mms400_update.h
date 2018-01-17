#ifndef __MMS400_UPDATE_H
#define __MMS400_UPDATE_H
//#include "sys.h"
#include "touch.h"
//#include <stdbool.h>
//#include <stddef.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_timer.h"
#include "nrf_delay.h"

#define  UPDATE_LOG_ENABLE               0

//ISC Info
#define ISC_PAGE_SIZE				128

//ISC Command
#define ISC_CMD_ERASE_ALL			{0xFB,0x4A,0x00,0x15,0x00,0x00}
#define ISC_CMD_ERASE_PAGE		{0xFB,0x4A,0x00,0x8F,0x00,0x00}
#define ISC_CMD_READ_PAGE			{0xFB,0x4A,0x00,0xC2,0x00,0x00}
#define ISC_CMD_WRITE_PAGE		{0xFB,0x4A,0x00,0xA5,0x00,0x00}
#define ISC_CMD_PROGRAM_PAGE		{0xFB,0x4A,0x00,0x54,0x00,0x00}
#define ISC_CMD_READ_STATUS		{0xFB,0x4A,0x36,0xC2,0x00,0x00}
#define ISC_CMD_EXIT				{0xFB,0x4A,0x00,0x66,0x00,0x00}

//ISC Status
#define ISC_STATUS_BUSY				0x96
#define ISC_STATUS_DONE				0xAD

#define MIP_BIN_TAIL_MARK		{0x4D, 0x42, 0x54, 0x01}	// M B T 0x01
#define MIP_BIN_TAIL_SIZE		64


/**
* Firmware binary tail info
*/
struct mip_bin_tail {
	u8 tail_mark[4];
	char chip_name[4];
	u32 bin_start_addr;
	u32 bin_length;

	u16 ver_boot;
	u16 ver_core;
	u16 ver_app;
	u16 ver_param;
	u8 boot_start;
	u8 boot_end;
	u8 core_start;
	u8 core_end;
	u8 app_start;
	u8 app_end;
	u8 param_start;
	u8 param_end;

	u8 checksum_type;
	u8 hw_category;
	u16 param_id;
	u32 param_length;
	u32 build_date;
	u32 build_time;

	u32 reserved1;
	u32 reserved2;
	u16 reserved3;
	u16 tail_size;
	u32 crc;
} __attribute__ ((packed));

struct mms_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;
} __attribute__ ((packed));

enum fw_update_errno{
       fw_err_retries_over = -5,
	fw_err_file_read = -4,
	fw_err_file_open = -3,
	fw_err_file_type = -2,
	fw_err_download = -1,
	fw_err_none = 0,
	fw_err_uptodate = 1,
};

struct firmware {
    size_t size;
    const u8 *data;
};
static int mms_isc_read_status(u8 addr);
static int mms_isc_erase_page(u8 addr);
static int mms_isc_program_page(u8 addr, int offset,const u8 *data, int length);
static int mms_isc_read_page(u8 addr,int offset, u8 *data);
static int mms_isc_exit(u8 addr);
int mms_flash_fw(u8 Addr, const u8 *fw_data, u16 fw_size, bool force, bool section);
#endif
