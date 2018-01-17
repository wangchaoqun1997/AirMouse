#include "sys.h"		
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "i2c.h"
#include "mms400_update.h"


/*********************************
*************PA1  INT*************
*************PA2  RST*************
*************PB6  SCL*************
*************PB7  SDA*************
***************I2C1***************/
#if    0
u8 EVENT_CATEGORY[2] = {0,0},TX2_Buffer[BUFFERSIZE] = {0x02,},RX2_Buffer[BUFFERSIZE ] = {0,0,0,0,0,0,0,0,};
u8 eventsize = 0 ,event_category = 0,alert_event = 0,gesture_code = 0;
#endif
extern Touch_Event Touch_Info;



int main (void)
{
#if     (AUTO_UPDATE_ENABLE)
	int ret;
#endif

#if   (I2C_READ_MODE == 0) 
	u8 wbuf[3];
#endif
	
	const extern u8 MELFAS_MFSB[];
	extern struct firmware melfas_fw;
	
	Stm32_Clock_Init(9);
	
	uart_init(72, 921600);
	delay_init(72);
	 USART1_printf("("__DATE__ " - " __TIME__ ") \r\n");
	LED_Init();
  	I2C_Config(I2C_Mode_I2C, NORMAL_SPEED, I2C_AcknowledgedAddress_7bit);
	Touch_Init();

       #if     (AUTO_UPDATE_ENABLE)
	ret = mms_flash_fw(TOUCH_DEVICE_ADDR, melfas_fw.data,melfas_fw.size,false,true);
	if(ret <0 )
		USART1_printf("mms_flash_fw failed \r\n");
	#endif

	Touch_Exti_Init();
	
#if   (I2C_READ_MODE == 0)       //I2C_READ_MODE:  0------polling mode        1---------interrupt mode    defined in touch.h
       wbuf[0] = MIP_R0_STATUS;
   	wbuf[1] = MIP_R1_STATUS_TRIGGER_TYPE;
	wbuf[2] = 0x02;
	mms_i2c_write(TOUCH_DEVICE_ADDR,wbuf,3);
	USART1_printf("polling mode setting done \r\n");
#endif

	USART1_printf("Init finish \r\n");
	while(1)
	{
	
	#if     (I2C_READ_MODE == 0)     //I2C_READ_MODE:  0------polling mode        1---------interrupt mode  defined in touch.h
       u8 i;
	u8 EVENT_CATEGORY[2] = {0,0},RX2_Buffer[BUFFERSIZE ] = {0,0,0,0,0,0,0,0,};
       u8 eventsize = 0 ,event_category = 0,alert_event = 0,gesture_code = 0;

   	wbuf[0] = MIP_R0_STATUS;
   	wbuf[1] = MIP_R1_STATUS_EVENT_READY;

		mms_i2c_read(TOUCH_DEVICE_ADDR,wbuf,2,RX2_Buffer,1);
	
		switch(RX2_Buffer[0])
		{
			case 0x00:
				break;
			case 0x01:
				  wbuf[0] = MIP_R0_EVENT;
   	                       wbuf[1] = MIP_R1_EVENT_PACKET_INFO;

				  event_category = (EVENT_CATEGORY[0] >> 7) & 0x01;  // 0:normal touch  others:alert event
				  eventsize = EVENT_CATEGORY[0] & 0x7F;            // finger number,each finger size:6 bytes
				  
				  wbuf[0] = MIP_R0_EVENT;
   	      			  wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
				  if(event_category ==0)
				  {
					 
					  if(eventsize > 0)
					  {
						  mms_i2c_read(TOUCH_DEVICE_ADDR,wbuf,2,RX2_Buffer,eventsize);
						  for(i=0;i<eventsize/6;i++)
						  {
				              	 Touch_Info.Finger_ID = RX2_Buffer[0 + i*6] & 0x0F -1;
							  Touch_Info.Touch_Status = RX2_Buffer[0 + i*6] >>7;  // 0:touch off/release  1:touch on/hold on
							  Touch_Info.X_Axis = ((RX2_Buffer[1 + i*6] << 8) & 0x0F00)|RX2_Buffer[2 + i*6];
							  Touch_Info.Y_Axis = ((RX2_Buffer[1 + i*6] << 4) & 0x0F00)|RX2_Buffer[3 + i*6];

							 USART1_printf("TOUCH INFO: Finger_ID= %d,Touch_Status= %d,X_Axis= %d,Y_Axis= %d\r\n",\
			   					Touch_Info.Finger_ID,Touch_Info.Touch_Status,Touch_Info.X_Axis,Touch_Info.Y_Axis);
						  }
						  USART1_printf("Read Sucess \r\n");
					  }
				  }
				  else // gesture recognizing
				  {
						mms_i2c_read(TOUCH_DEVICE_ADDR,wbuf,2,RX2_Buffer,2);
						alert_event = RX2_Buffer[0];
						if(alert_event == 2) //gesture recognize
						{
							gesture_code = RX2_Buffer[1];
							switch(gesture_code)  //Here is need modified if any customising
							{
								case MIP_EVENT_GESTURE_FLICK_RIGHT:	
									USART1_printf("Gesture info:Slide From Left To Right\r\n");
									break;
								case MIP_EVENT_GESTURE_FLICK_DOWN:
								    USART1_printf("Gesture info:Slide From Top To Bottom\r\n");
									break;
								case MIP_EVENT_GESTURE_FLICK_LEFT:
									USART1_printf("Gesture info:Slide From Right To Left\r\n");
									break;
								case MIP_EVENT_GESTURE_FLICK_UP:
									USART1_printf("Gesture info:Slide_From Bottom To Top\r\n");
									break;
								case MIP_EVENT_GESTURE_DOUBLE_TAP:
									USART1_printf("Gesture info:Double Tap\r\n");
									break;
								default:
									USART1_printf("No gesture recognizing\r\n");
									break;
									
							}
						}
						
				  	}
				  break;
			default:
				break;
		}
		
#endif
	}
	
}
