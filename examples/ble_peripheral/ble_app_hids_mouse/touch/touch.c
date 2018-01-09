#include "touch.h"
#include "usart.h"
#include "delay.h"
#if   0
extern  u8 RX2_Buffer[BUFFERSIZE];// eventsize,
extern  u8 EVENT_CATEGORY[2];
extern  u8 eventsize, event_category,alert_event,gesture_code;
extern  u8 EVENT_CATEGORY[2];
#endif
Touch_Event Touch_Info;

void MMS_Reboot(void)
{
    	GPIOA->CRL &= 0xFFFFF0FF;//PA1 INT  pull up input,PA2 RST pull up output
    	GPIOA->CRL |= 0x00000300;
		
	GPIOA->ODR &= ~0x04;   //RST action;
	delay_ms(5);
	GPIOA->ODR |= 0x04;
	delay_ms(30);
}

void Touch_Init(void)
{
	RCC->APB2ENR|=1<<2;   //enable portA clock
    	GPIOA->CRL &= 0xFFFFF00F;//PA1 INT  pull up input,PA2 RST pull up output
    	GPIOA->CRL |= 0x00000380;
	GPIOA->ODR |= 1 << 1;

	/*GPIOA->ODR &= ~0x04;   //RST action;
	delay_ms(5);
	GPIOA->ODR |= 0x04;
	delay_ms(30);*/

	MMS_Reboot();
}

void Touch_Exti_Init(void)
{
	u32 temp,temp1;
   
	RCC->APB2ENR|=1 << 0;   //enbale AFIO clocclk

	AFIO->EXTICR[0] &= ~(0x000F << 4);//clear the set up first
    AFIO->EXTICR[0] |= 0x0000;//PA1 EXIT
	EXTI->IMR |= 1 << 1; //enable IRQ from line1
	EXTI->FTSR |= 1 << 1;  //enable line1 falling trigger

		  
	temp1=(~0x02)&0x07;//取后三位
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
	temp|=0X05FA0000; //写入钥匙
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组

	temp=2 <<(4-2);	  
	temp|=2&(0x0f>>2);
	temp&=0xf;//取低四位  
	//if(7<32)NVIC->ISER[0]|=1<<7;//使能中断位(要清除的话,相反操作就OK)
	//else NVIC->ISER[1]|=1<<(7-32);
       NVIC->ISER[0]|=1<<7;	
	NVIC->IP[7]|=temp<<4;//设置响应优先级和抢断优先级
}

int mms_i2c_write(u8 device, u8 *write_buf, u16 write_len)
{
       int ret;
	ret = I2C_MasterWtiteMultiBytes(device,write_buf,write_len);

	if(ret == 0)
	{
		goto EXIT;
	}
	else if(ret < 0)
	{
		USART1_printf("[ERROR] i2c_write - errno[%d]\r\n", ret);
	}
	else
	{
		USART1_printf("[ERROR] unknown error[%d]\r\n", ret);
	}
	goto ERROR;
ERROR:
	//USART1_printf("[ERROR] mms_i2c_write failed\r\n");
	return 1;
EXIT:
	return 0;
}

int mms_i2c_read(u8 device, u8 *write_buf, u16 write_len, u8 *read_buf, u16 read_len)
{
	int ret,res;
	ret = mms_i2c_write(device,write_buf,write_len);
	if(ret == 1)
	{
	 	USART1_printf("[ERROR] mms_i2c_write failed\r\n");
		goto ERROR;
	}
	res= I2C_MasterReadMultiBytes(device,read_buf,read_len);
	
	if(res == 0)
	{
		goto EXIT;
	}
	else if(res < 0)
	{
		USART1_printf("[ERROR] i2c_read- errno[%d]\r\n", ret);
	}
	else
	{
		USART1_printf("[ERROR] unknown error[%d]\r\n", ret);
	}
	goto ERROR;

ERROR:
	return 1;
EXIT:
	return 0;
}

int    MMS_Get_FW_Version(u8 Addr,u8 *ver_buf)
{
	u8 rbuf[8];
	u8 wbuf[2];
	u8 i;

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_BOOT;
	
	if(mms_i2c_read(Addr,wbuf,2,rbuf,8))
	{
		goto ERROR;
	}

	for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++)
	{
		ver_buf[0 + i * 2] = rbuf[1 + i * 2];
		ver_buf[1 + i * 2] = rbuf[0 + i * 2];
	}

	return 0;
ERROR:
	USART1_printf("[ERROR]MMS_Get_FW_Version failed\r\n");
	return 1;	
	
}

int MMS_Get_FW_Version_u16(u8 Addr,u16 *ver_buf_u16)
{
	u8 rbuf[8];
	u8 i;

	if(MMS_Get_FW_Version(Addr,rbuf))
	{
		goto ERROR;
	}

	for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++)
	{
		ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];
	}	
	
	return 0;
ERROR:
	USART1_printf(" [ERROR]MMS_Get_FW_Version_u16\r\n");
	return 1;	

}


void EXTI1_IRQHandler(void)
{
#if           1
   u8 i; 
   u8 wbuf[3];
   u8 EVENT_CATEGORY[2] = {0,0},RX2_Buffer[BUFFERSIZE ] = {0,0,0,0,0,0,0,0,};
   u8 eventsize = 0 ,event_category = 0,alert_event = 0,gesture_code = 0;
  wbuf[0] = MIP_R0_EVENT;
  wbuf[1] = MIP_R1_EVENT_PACKET_INFO;

  mms_i2c_read(TOUCH_DEVICE_ADDR,wbuf,2,RX2_Buffer,1);
  EVENT_CATEGORY[0] = RX2_Buffer[0];
  //USART1_printf("EVENT_CATEGORY[0] = 0x%x \r\n", EVENT_CATEGORY[0]);
  event_category = (EVENT_CATEGORY[0] >> 7) & 0x01;  // 0:normal touch  others:alert event
  eventsize = EVENT_CATEGORY[0] & 0x7F;            // finger number,each finger size:6 bytes
  if(event_category ==0)
  {
	  if(eventsize > 0)
	  {
		  wbuf[0] = MIP_R0_EVENT;
   		  wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
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
	
	EXTI->PR=1<<1;  //清除LINE0上的中断标志位  
	#endif
}

