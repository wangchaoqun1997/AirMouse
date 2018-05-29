/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "sw3153_driver.h"
#define SAMPLES_IN_BUFFER 5
int saadc_sample_time_ms=20;
volatile uint8_t state = 1;

static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint32_t              m_adc_evt_counter;
#define BATTERY_BUFFER 5
#define BATTERY_BUFFER_ANALYZE 10
#define BATTERY_BUFFER_SLOW 20
char battery_level=50;
static nrf_saadc_value_t battery_buffer[BATTERY_BUFFER];
static nrf_saadc_value_t battery_buffer_slow[BATTERY_BUFFER_SLOW];
static nrf_saadc_value_t battery_level_analyze[BATTERY_BUFFER_ANALYZE];
static nrf_saadc_value_t battery_level_adc,battery_level_adc_laster=0;
static nrf_saadc_value_t battery_level_adc_slow,battery_level_adc_laster_slow=0;
#define R301 47
#define R302 100
#define BATTERY_FULL 820 //(int)((4.32*(R302/(R301+R302)))*(1023/3.6))
#define BATTERY_LOW  550 //(int)((2.60*(R302/(R301+R302)))*(1023/3.6))   193.23
enum status_flag_{
CHARGER_PLUG_IN=0x10,
CHARGER_PLUG_IN_3V,
CHARGER_PLUG_OUT,
CHARGER_BATTERY_FULL,
CHARGER_BATTERY_LOW,
CHARGER_BATTERY_NOMAL,
CHARGER_BATTERY_UNKONW,
};
static bool plug_in_flag=false;
static enum status_flag_ status_flag=CHARGER_BATTERY_NOMAL;
static enum status_flag_ status_flag_lastest=CHARGER_BATTERY_NOMAL;
#include "ble_bas.h"
extern ble_bas_t      m_bas; 
extern uint16_t       m_conn_handle ;
#include "app_timer.h"
APP_TIMER_DEF(battery_led_id);
//--saadc oper start
static void battery_led_start(void)
{
	app_timer_start(battery_led_id,APP_TIMER_TICKS(10),NULL);//8min
}
#define BATTERY_CAPCITY 520
static enum status_flag_ analyze_plug_status(nrf_saadc_value_t *battery_level_trend)
{

	//nrf_saadc_value_t battery_init_adc=(battery_level_trend[2]+battery_level_trend[1]+battery_level_trend[0])/3;
		if((battery_level_trend[1]>=battery_level_trend[0]) &&(battery_level_trend[2]>=battery_level_trend[1]) &&(battery_level_trend[3]>=battery_level_trend[2])&&(battery_level_trend[4]>=battery_level_trend[3])&&((battery_level_trend[4]-3)>=battery_level_trend[0])){
				NRF_LOG_INFO("charger plug in !!!! \r\n");
				status_flag = CHARGER_PLUG_IN;
				battery_led_start();
		}else{
				//NRF_LOG_INFO("charger plug unknow !!!! \r\n");
		}
#if 0
	if(battery_init_adc <=750 || (battery_init_adc>750 && battery_level_trend[0]<=760)){
		nrf_saadc_value_t temp1=0,temp2=0;
		for(int i=0;i<5;i++){
			temp1 +=battery_level_trend[i];
			temp2 +=battery_level_trend[i+5];
		}
		temp1 =(temp1/5)+1;
		temp2 =(temp2/5);
		if((temp2 > temp1) && battery_level_trend[BATTERY_BUFFER_ANALYZE-1]>800){
		NRF_LOG_INFO("charger plug temp1[%d] temp2[%d]------------ \r\n",temp1,temp2);
			for(int i=0;i<(BATTERY_BUFFER_ANALYZE-1);i++){
				if(battery_level_trend[i+1]>=(battery_level_trend[i]) || (temp2-temp1)>2 ){
					if(i==(BATTERY_BUFFER_ANALYZE-2)) {
						NRF_LOG_INFO("charger plug in !!!! \r\n");
						status_flag = CHARGER_PLUG_IN;
						battery_led_start();
					}
				}else{
					NRF_LOG_INFO("charger plug unknow !!!! \r\n");
					break;
				}
			}
		}
	}else{

		if((battery_level_trend[4]>=battery_level_trend[3]) &&  (battery_level_trend[2] > battery_level_trend[1]) && (battery_level_trend[5]>=battery_level_trend[4]) && (battery_level_trend[1]>battery_level_trend[0]) && battery_level_trend[BATTERY_BUFFER_ANALYZE-1]>830){
		NRF_LOG_INFO("charger plug in !!!! \r\n");
		status_flag = CHARGER_PLUG_IN;
		battery_led_start();
					NRF_LOG_INFO("--------------battery status CHARGER_PLUG_IN\r\n");
		}else if((battery_level_trend[4]<=battery_level_trend[3]) &&  (battery_level_trend[2] < battery_level_trend[1]) && (battery_level_trend[5]<=battery_level_trend[4]) && (battery_level_trend[1]<battery_level_trend[0])){
		NRF_LOG_INFO("charger plug out !!!! \r\n");
		status_flag = CHARGER_PLUG_OUT;								
		battery_led_start();
					NRF_LOG_INFO("--------------battery status CHARGER_PLUG_OUT\r\n");
		}else{
		NRF_LOG_INFO("charger plug unknow !!!! \r\n");
		}
	}

#endif
}

extern char project_flag;
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

				static nrf_saadc_value_t sample_avg=0;

        for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            //NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[i]);
						sample_avg +=p_event->data.done.p_buffer[i];
        }
				
				sample_avg /=SAMPLES_IN_BUFFER;
				
				//NRF_LOG_INFO("sample_avg %d\r\n",sample_avg);
//----start

				static int init_flag=0;
				if(init_flag == 0){
					init_flag =1;
					for(int i=0;i<BATTERY_BUFFER;i++){
						battery_buffer[i]=sample_avg;
					}
					for(int i=0;i<BATTERY_BUFFER_SLOW;i++){
						battery_buffer_slow[i]=sample_avg;
					}
				}
////////------------------
				static int slowness=0;//200ms
				static char time_slow=0;
				if(time_slow == BATTERY_BUFFER_SLOW)
					time_slow = 0;
				slowness++;
				if(slowness >= 1000){//100*1800 =180 000ms
					slowness = 0;
					battery_buffer_slow[time_slow++]=sample_avg;
				}
				battery_level_adc_laster_slow = battery_level_adc_slow;
				battery_level_adc_slow=0;				
				for(int i=0;i<BATTERY_BUFFER_SLOW;i++){
					battery_level_adc_slow+=battery_buffer_slow[i];
					//NRF_LOG_INFO(" battery_buffer_slow[%d] %d\r\n",i,battery_buffer_slow[i]);
				}
				battery_level_adc_slow /=BATTERY_BUFFER_SLOW;
				if(battery_level_adc_laster_slow == 0){
					battery_level_adc_laster_slow = battery_level_adc_slow;
				}
				//NRF_LOG_INFO("battery_level_adc_laster_slow[%d] battery_level_adc_slow[%d] ----\r\n",battery_level_adc_laster_slow,battery_level_adc_slow);
				static int level_updata=0;
				level_updata++;
				if(level_updata >60){
					level_updata=0;
					//battery_level =(char)( (battery_level_adc_slow*0.119));
					if(project_flag == 0x03){
						if(battery_level_adc_slow<30)
								battery_level_adc_slow = 30;
						battery_level =(char)( ((battery_level_adc_slow-30)*0.34));
					}else{
						if(battery_level_adc_slow<BATTERY_LOW)
								battery_level_adc_slow = BATTERY_LOW;
						battery_level =(char)( ((battery_level_adc_slow-BATTERY_LOW)*0.34));	
					}
					if(battery_level>=88)
						battery_level=100;
					if(battery_level<=0)
						battery_level=1;
					//NRF_LOG_INFO(" battery_level[%d] battery_level_adc_slow[%d]\r\n",battery_level,battery_level_adc_slow);
					//custom_on_send(m_conn_handle,&m_bas,temp1,sizeof(temp1));
				}
////////------------------
				static char time=0;
				if(time == BATTERY_BUFFER)
					time = 0;

				battery_buffer[time++]=sample_avg;
				battery_level_adc_laster = battery_level_adc;
				battery_level_adc=0;				
				for(int i=0;i<BATTERY_BUFFER;i++){
					battery_level_adc+=battery_buffer[i];
					//NRF_LOG_INFO(" battery_buffer[%d] %d\r\n",i,battery_buffer[i]);
				}
				battery_level_adc /=BATTERY_BUFFER;
				if(battery_level_adc_laster == 0)
					battery_level_adc_laster = battery_level_adc;
				//NRF_LOG_INFO("battery_level_adc_laster[%d] battery_level_adc[%d] ----\r\n",battery_level_adc_laster,battery_level_adc);

//-------------				
				static char flag=0;
				static bool start_sample=false,sampling=false;
				if(
				#if 0
				  (
				   (((battery_level_adc - battery_level_adc_laster)>10 || (battery_level_adc_laster - battery_level_adc)>10) && battery_level_adc <= 800)
				   ||
				   (((battery_level_adc - battery_level_adc_laster)>4 || (battery_level_adc_laster - battery_level_adc)>4) && battery_level_adc > 800)
				  )	
				#else
				   ((battery_level_adc - battery_level_adc_laster)>=1)
				#endif
				   && 
				  start_sample==false 
				  ){
					//NRF_LOG_INFO("start_sample----------- \r\n");
					start_sample=true;
				}else if(battery_level==100/*battery_level_adc_slow >= (BATTERY_FULL)*/){
					status_flag = CHARGER_BATTERY_FULL;
				}else if(battery_level<=20/*battery_level_adc_slow <= (680BATTERY_LOW)*/){
					status_flag = CHARGER_BATTERY_LOW;
					battery_led_start();
				}else{
					status_flag = CHARGER_BATTERY_NOMAL;			
				}
				
				if(start_sample==true){

					battery_level_analyze[flag++]=battery_level_adc;

					if(flag == BATTERY_BUFFER_ANALYZE){
						//NRF_LOG_INFO("start_sample ok1[%d][%d][%d][%d][%d],analyzeing ----------- \r\n",\
										battery_level_analyze[0],battery_level_analyze[1],battery_level_analyze[2],battery_level_analyze[3],battery_level_analyze[4]);
						//NRF_LOG_INFO("start_sample ok2[%d][%d][%d][%d][%d]\r\n",\
										battery_level_analyze[5],battery_level_analyze[6],battery_level_analyze[7],battery_level_analyze[8],battery_level_analyze[9]);
						analyze_plug_status(battery_level_analyze);	 
						flag=0;
						start_sample = false;
						for(int i=0;i<BATTERY_BUFFER_ANALYZE;i++){battery_level_analyze[i]=0;}
					}

				}

				

//----end
			if(status_flag_lastest != status_flag){

				status_flag_lastest = status_flag;
				battery_led_start();

				if(CHARGER_PLUG_OUT == status_flag){
				}else if(CHARGER_PLUG_IN == status_flag){
					NRF_LOG_INFO("--------------battery status CHARGER_BATTERY_IN\r\n");
				}else if(CHARGER_BATTERY_NOMAL == status_flag){
					NRF_LOG_INFO("--------------battery status CHARGER_BATTERY_NOMAL\r\n");
				}else if(CHARGER_BATTERY_LOW == status_flag){
					NRF_LOG_INFO("--------------battery status CHARGER_BATTERY_LOW\r\n");
				}else if(CHARGER_BATTERY_FULL == status_flag){
					NRF_LOG_INFO("--------------battery status CHARGER_BATTERY_FULL\r\n");
				}else if(CHARGER_PLUG_IN_3V == status_flag){
					NRF_LOG_INFO("--------------battery status CHARGER_PLUG_IN_3V\r\n");
				}
			}	
				sample_avg=0;
    }

}
extern bool Mode_2D;
extern bool Mode_3D;
extern bool Mode_test;
extern bool Mode_mouse;
void led_reset()
{
	if(Mode_2D){
		if(Mode_mouse == true){
			sw3153_light_select(BLUE_GREEN,BLINK_LEVEL_NON);
		}else{
			sw3153_light_select(BLUE,BLINK_LEVEL_NON);
		}
	}else if(Mode_3D){
		if(Mode_mouse == true){
			//sw3153_blue_green();
		}else{
			sw3153_light_select(GREEN,BLINK_LEVEL_NON);
		}	
	}else if(Mode_test){
			sw3153_light_select(RED,BLINK_LEVEL_2);
	}else{
			sw3153_light_select(BLUE,BLINK_LEVEL_2);
	}

}


static void battery_led_stop(void)
{
	app_timer_stop(battery_led_id);
}
static void battery_led_handler(void* p_context)
{
	//NRF_LOG_INFO("battery_led_handler -------------\r\n");
	if(CHARGER_PLUG_IN == status_flag){
		sw3153_light_select(GREEN,BLINK_LEVEL_0);
		nrf_delay_ms(2000);
		led_reset();
		plug_in_flag=true;
	}else if(CHARGER_PLUG_OUT == status_flag){
			led_reset();
		plug_in_flag=false;
	}else if(CHARGER_BATTERY_NOMAL == status_flag){
			led_reset();
	}else if(CHARGER_BATTERY_LOW == status_flag){
		sw3153_light_select(RED,BLINK_LEVEL_1);
	}else if(CHARGER_BATTERY_FULL == status_flag){

		if(plug_in_flag == true){
		sw3153_light_select(GREEN,BLINK_LEVEL_2);
			nrf_delay_ms(2000);
			led_reset();
			plug_in_flag=false;
		}
	}

	//battery_led_start();
}
static void battery_led_init(void)
{
	NRF_LOG_INFO("battery_led_init -------------\r\n");
	app_timer_create(&battery_led_id,APP_TIMER_MODE_SINGLE_SHOT,battery_led_handler);
}
void saadc_init(void)
{
    NRF_LOG_INFO("SAADC HAL simple example.\r\n");
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
	
		battery_led_init();
		//battery_led_start();

}
/**
 * @brief Function for main application entry.
 */
void main_saadc_init(void)
{


    NRF_LOG_INFO("SAADC HAL simple example.\r\n");
    saadc_init();
    //saadc_sampling_event_init();
	nrf_drv_saadc_sample();
    //NRF_LOG_INFO("SAADC HAL simple example.end\r\n");
    //saadc_sampling_event_enable();


}


/** @} */
