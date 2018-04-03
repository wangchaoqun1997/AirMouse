#include "sw3153_driver.h"
#include "nrf_drv_twi.h"


extern const nrf_drv_twi_t m_twi;
static int I2C_Write_Addr8(	const uint8_t slave_addr,uint8_t write_addr,uint8_t write_value)
{
		unsigned char err_code;
	  unsigned char  write[2]={write_addr,write_value};	
	  err_code = nrf_drv_twi_tx(&m_twi, slave_addr, write, sizeof(write), false);
		return err_code;
}
void sw3153_config(void)
{
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_RESET, WD_LED_RESET_MASK);
	
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_GLOBAL_CONTROL, WD_LED_MOUDLE_ENABLE_MASK);
	
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE,   WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |WD_LED_BREATHE_MODE_MASK | OUT_CURRENT);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE+1, WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |WD_LED_BREATHE_MODE_MASK | OUT_CURRENT);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE+2, WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |WD_LED_BREATHE_MODE_MASK | OUT_CURRENT);
/*
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+0,   RISE_TIME<<4 | HOLD_TIME);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+1,   FALL_TIME<<4 | OFF_TIME);
		nrf_delay_us(8);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+3, RISE_TIME<<4 | HOLD_TIME);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+4, FALL_TIME<<4 | OFF_TIME);
		nrf_delay_us(8);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+6, RISE_TIME<<4 | HOLD_TIME);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+7, FALL_TIME<<4 | OFF_TIME);
		nrf_delay_us(8);
*/	
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_BRIGHTNESS_BASE,  0x02);//pwm
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_BRIGHTNESS_BASE+1, 0x02);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_BRIGHTNESS_BASE+2, 0x02);
		
		//I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_ENABLE, LED_BLUE);
}
static void sw3153_blink_off(void)
{
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE,   WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |0x00 | OUT_CURRENT);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE+1, WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |0x00 | OUT_CURRENT);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE+2, WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |0x00 | OUT_CURRENT);

}
static void sw3153_blink_on(void)
{
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE,   WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |WD_LED_BREATHE_MODE_MASK | OUT_CURRENT);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE+1, WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |WD_LED_BREATHE_MODE_MASK | OUT_CURRENT);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_CONFIG_BASE+2, WD_LED_FADE_OFF_MASK | WD_LED_FADE_ON_MASK |WD_LED_BREATHE_MODE_MASK | OUT_CURRENT);
}
static void sw3153_blink_on_set(uint8_t rise_time,uint8_t fall_time,uint8_t hold_time,uint8_t off_time)
{
		sw3153_blink_on();
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+0, rise_time<<4 | hold_time);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+1, fall_time<<4 | off_time);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+3, rise_time<<4 | hold_time);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+4, fall_time<<4 | off_time);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+6, rise_time<<4 | hold_time);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_TIMESET0_BASE+7, fall_time<<4 | off_time);
}
static void sw3153_green(void)
{
		sw3153_blink_off();
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_ENABLE, LED_GREEN);
}
static void sw3153_blue(void)
{
		sw3153_blink_off();
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_ENABLE, LED_BLUE);
}
static void sw3153_blue_green(void)
{
		sw3153_blink_off();
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_ENABLE, LED_BLUE|LED_GREEN);
}
static void sw3153_blue_green_red(void)
{
		sw3153_blink_off();
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_ENABLE, LED_BLUE|LED_GREEN|LED_RED);
}
static void sw3153_red(void)
{
		sw3153_blink_off();
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_ENABLE, LED_RED);
}
static void sw3153_off(void)
{
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_LED_ENABLE, 0x00);
		I2C_Write_Addr8(WD3153_ADDRESS,WD_REG_GLOBAL_CONTROL, 0x00);
}

void sw3153_light_select(enum sw3153_light_type type,enum sw3153_blink_level level)
{
	switch(type){
		case GREEN:
			sw3153_green();break;
		case RED:
			sw3153_red();break;
		case BLUE:
			sw3153_blue();break;
		case BLUE_GREEN:
			sw3153_blue_green();break;
		case BLUE_GREEN_RED:
			sw3153_blue_green_red();break;
		case OFF:
			sw3153_off();break;
	}

	switch(level){
		case BLINK_LEVEL_NON:
			break;
		case BLINK_LEVEL_0:
			sw3153_blink_on_set(0x00,0x00,0x00,0x00);break;
		case BLINK_LEVEL_1:
			sw3153_blink_on_set(0x01,0x01,0x01,0x01);break;
		case BLINK_LEVEL_2:
			sw3153_blink_on_set(0x02,0x02,0x02,0x02);break;
		case BLINK_LEVEL_3:
			sw3153_blink_on_set(0x03,0x03,0x03,0x03);break;
		case BLINK_LEVEL_4:
			sw3153_blink_on_set(0x04,0x04,0x04,0x04);break;
	}
}
