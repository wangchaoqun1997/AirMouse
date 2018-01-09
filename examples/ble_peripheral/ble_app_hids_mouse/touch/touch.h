#include "sys.h"
#include "i2c.h"

#define TOUCHPAD_MELFAS_MMS427    1
#define TOUCHPAD_MELFAS_MMS438    2
#define CHIP_MMS427	      427
#define CHIP_MMS438	      438
#define  TOUCH_IC_SELECT       TOUCHPAD_MELFAS_MMS427

#define I2C_READ_MODE                        1             // 0:using polling mode     1:using interrupt mode

#define AUTO_UPDATE_ENABLE                            0           //:0 disable update touch module firmware function   1:enable update touch module firmware function

#define MMS_USE_AUTO_FW_UPDATE	0	// 0 or 1
#define MMS_FW_MAX_SECT_NUM		4
#define MMS_FW_UPDATE_DEBUG		0	// 0 or 1
#define MMS_FW_UPDATE_SECTION	1	// 0 or 1
#define MMS_EXT_FW_FORCE_UPDATE	0	//0 or 1


#define TOUCH_DEVICE_ADDR        (0x48 << 1)     //melfas I2C address

#define MIP_EVENT_GESTURE_FLICK_RIGHT	20			
#define MIP_EVENT_GESTURE_FLICK_DOWN	       21		
#define MIP_EVENT_GESTURE_FLICK_LEFT	       22	
#define MIP_EVENT_GESTURE_FLICK_UP		23
#define MIP_EVENT_GESTURE_DOUBLE_TAP	       24

#define MIP_R0_INFO							0x01
#define MIP_R1_INFO_VERSION_BOOT			0x20

#define MIP_R0_EVENT						0x02
#define MIP_R1_EVENT_PACKET_INFO			0x10
#define MIP_R1_EVENT_PACKET_DATA			0x11

#define MIP_R0_STATUS                                       0x06
#define MIP_R1_STATUS_EVENT_READY		 0x01
#define MIP_R1_STATUS_TRIGGER_TYPE              0x11

#if !defined BUFFERSIZE
#define BUFFERSIZE              30
#endif

#if      1
typedef  struct  _Touch_Format{
  u8 Touch_Status;
  u8 Finger_ID;
  u16 X_Axis;
  u16 Y_Axis;
  u8 Z_Pressure;
  u8 Touch_Radius;
}Touch_Event,*PTouch_EVent;
#endif
void  MMS_Reboot(void);
void Touch_Init(void);
void Touch_Exti_Init(void);
int mms_i2c_write(u8 device, u8 *write_buf, u16 write_len);
int mms_i2c_read(u8 device, u8 *write_buf, u16 write_len, u8 *read_buf, u16 read_len);
int   MMS_Get_FW_Version(u8 Addr,u8 *ver_buf);
int MMS_Get_FW_Version_u16(u8 Addr,u16 *ver_buf_u16);

void EXTI1_IRQHandler(void);
