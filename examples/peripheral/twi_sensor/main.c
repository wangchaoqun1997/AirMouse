/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define LM75B_ADDR          (0x45U)

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

#define WD_REG_RESET            0x00
#define WD_REG_GLOBAL_CONTROL       0x01
#define WD_REG_LED_STATUS       0x02
#define WD_REG_LED_ENABLE       0x30
#define WD_REG_LED_CONFIG_BASE      0x31
#define WD_REG_LED_BRIGHTNESS_BASE  0x34
#define WD_REG_TIMESET0_BASE        0x37
#define WD_REG_TIMESET1_BASE        0x38

#define WD3153_CHIPID           0x33
#define WD_LED_MOUDLE_ENABLE_MASK   0x01
#define WD_LED_FADE_OFF_MASK        0x40
#define WD_LED_FADE_ON_MASK     0x20
#define WD_LED_BREATHE_MODE_MASK    0x10
#define WD_LED_RESET_MASK       0x55


	char gpio_num = 11;
/* Mode for LM75B. */
#define NORMAL_MODE 0U

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;
static uint8_t m_sample_1;
/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
static void  twi_tx(uint8_t reg,uint8_t value)
{
		ret_code_t err_code;
		m_xfer_done = false;

		uint8_t reg1[2] = {reg, value};
		err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg1, sizeof(reg1), false);
		APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);

}

static void  twi_rx(const uint8_t reg,uint8_t * value)
{
		ret_code_t err_code;
		m_xfer_done = false;

		err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, &reg, 1, false);
		APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		
		m_xfer_done = false;
		err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, value, sizeof(uint8_t));
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);

}

void LM75B_set_mode(void)
{

		twi_tx(0x00, 0x55);
		nrf_delay_us(8);

		twi_tx(WD_REG_GLOBAL_CONTROL, WD_LED_MOUDLE_ENABLE_MASK);
		nrf_delay_us(8);
		twi_tx(0x31, 0x02);
		nrf_delay_us(8);
		twi_rx(0x31,& m_sample_1);
    	
		twi_tx(0x34, 0x22);
		nrf_delay_us(8);
		twi_rx(0x34,& m_sample_1);
	
		twi_tx(0x30, 0x07);
		nrf_delay_us(8);
		//twi_rx(0x30,& m_sample_1);
	
/*
		
		m_xfer_done = false;
	  err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample_1, sizeof(m_sample_1));
 APP_ERROR_CHECK(err_code);
 while (m_xfer_done == false);
			
	
		m_xfer_done = false;
    uint8_t reg1[2] = {0x00, 0x55};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg1, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);	

nrf_delay_us(8)	;
	
			m_xfer_done = false;
    uint8_t  reg2[2] = {WD_REG_GLOBAL_CONTROL, WD_LED_MOUDLE_ENABLE_MASK};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg2, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);		
nrf_delay_us(8)		;
			m_xfer_done = false;
     uint8_t reg3[2] = {0x31, 1};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg3, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);			

nrf_delay_us(8)	;	
			m_xfer_done = false;
     uint8_t reg4[2] = {0x34, 0xff};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg4, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

	nrf_delay_us(8)	;	
			m_xfer_done = false;
     uint8_t reg5[2] = {0x30, 0x07};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg5, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
*/		
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.\r\n", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
						

            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}
#include "nrf_drv_gpiote.h"
#define PIN_IN 11
 #define PIN_OUT 12
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_out_toggle(PIN_OUT);
}
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}
/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    gpio_init();
	//nrf_gpio_cfg_output(gpio_num);
	//nrf_gpio_pin_write(gpio_num,1);
    NRF_LOG_INFO("\r\nTWI sensor example\r\n");
    NRF_LOG_FLUSH();
    twi_init();
    LM75B_set_mode();



    while (true)
    {
        nrf_delay_ms(500);

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        read_sensor_data();
        NRF_LOG_FLUSH();
    }
}

/** @} */
