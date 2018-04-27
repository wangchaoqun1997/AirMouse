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
 *
 * @defgroup ble_sdk_app_hids_mouse_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_mouse
 * @brief HID Mouse Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Service for implementing a simple mouse functionality. This application uses the
 * @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device. This implementation of the
 * application will not know whether a connected central is a known device or not.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "softdevice_handler_appsh.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "app_button.h"
#include "ble_advertising.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "sw3153_driver.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#if BUTTONS_NUMBER < 4
#error "Not enough resources on board to run example"
#endif


#define DEVICE_NAME                     "AIR MOUSE"                              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "wangcq327_v1.0.3"  //vX.X.X  0<=X<=9  the max version is wangcq327_v9.9.9_...                     /**< Manufacturer. Will be passed to Device Information Service. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                        /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                        /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                      /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                      /**< Product Version. */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
#define SLAVE_LATENCY                   20                                         /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              5                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_CUSTOM1_LEN      1
#define INPUT_REP_CUSTOM2_LEN      1
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                           /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_CUSTOM1_INDEX    3
#define INPUT_REP_CUSTOM2_INDEX    4
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                           /**< Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_REF_CUSTOM1_ID   4
#define INPUT_REP_REF_CUSTOM2_ID   5

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, \
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE)       /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define APP_ADV_FAST_INTERVAL           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT            30                                          /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT            180                                         /**< The duration of the slow advertising period (in seconds). */


static ble_hids_t     m_hids;                                                       /**< Structure used to identify the HID service. */
ble_bas_t      m_bas;                                                        /**< Structure used to identify the battery service. */
static nrf_ble_gatt_t m_gatt;
static bool           m_in_boot_mode = false;                                       /**< Current protocol mode. */
uint16_t       m_conn_handle  = BLE_CONN_HANDLE_INVALID;                     /**< Handle of the current connection. */

static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */

APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */

static pm_peer_id_t m_peer_id;                                                      /**< Device reference handle to the current bonded central. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static pm_peer_id_t   m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];          /**< List of peers currently in the whitelist. */
static uint32_t       m_whitelist_peer_cnt;                                         /**< Number of peers currently in the whitelist. */
static bool           m_is_wl_changed;                                              /**< Indicates if the whitelist has been changed since last time it has been updated in the Peer Manager. */
//--------------dfu start

#include "ble_dfu.h"
static ble_dfu_t      m_dfus;                                                       /**< Structure used to identify the DFU service. */
static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_DFU_EVT_INDICATION_DISABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is disabled.\r\n");
            break;

        case BLE_DFU_EVT_INDICATION_ENABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is enabled.\r\n");
            break;

        case BLE_DFU_EVT_ENTERING_BOOTLOADER:
            NRF_LOG_INFO("Device is requested to enter bootloader mode!\r\n");
            break;

        default:
            NRF_LOG_INFO("Unknown event from ble_dfu.\r\n");
            break;
    }
}
APP_TIMER_DEF(touch_timer_id);
APP_TIMER_DEF(mouse_slow_id);
APP_TIMER_DEF(sensor_cal_id);
APP_TIMER_DEF(connect_sleep_id);
APP_TIMER_DEF(saadc_sample_id);
APP_TIMER_DEF(sensor_poll_timer_id);
APP_TIMER_DEF(touch_action_detect_id);


//--------------dfu end
ble_gatts_char_handles_t custom_char_handles;
uint8_t report_key;
uint8_t report_key_cache;
uint8_t report_key_touch;
uint8_t report_mouse=0x01;
static uint8_t m_sample_key_press_scan_str[] = /**< Key pattern to be sent when the key press button has been pushed. */
{
    0x0b,      //home                                /* Key h */
    0x08,                                      /* Key e */
    0x0f,//down                                      /* Key l */
    0x0f,                                      /* Key l */
    0x12,                                      /* Key o */
    0x28                                       /* Key Return */
};
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!\r\n");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t ret;

        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

        peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(ret);

        // Setup the device identies list.
        // Some SoftDevices do not support this feature.
        ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (ret != NRF_ERROR_NOT_SUPPORTED)
        {
            APP_ERROR_CHECK(ret);
        }

        m_is_wl_changed = false;

        ret = ble_advertising_start(BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);

            m_peer_id = p_evt->peer_id;

            // Note: You should check on what kind of white list policy your application should use.
            if (p_evt->params.conn_sec_succeeded.procedure == PM_LINK_SECURED_PROCEDURE_BONDING)
            {
                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible\r\n");
                NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d\r\n",
                               m_whitelist_peer_cnt + 1,
                               BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;
                    m_is_wl_changed = true;
                }
            }
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
extern char battery_level;
static void battery_level_update(void)
{
    ret_code_t err_code;
    //uint8_t  battery_level;

    //battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        //APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}
/****************add start **********/

 #define USE_SHADOW_CREATE
 #ifdef USE_SHADOW_CREATE
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#define K_RIGHT 0x4F //right
#define K_LEFT  0x50 //left
#define K_DOWN  0x51 //down
#define K_UP    0x52 //up
#define K_ENTER 0x28 //right
#define K_VOL_UP (0x01<<5) //right
#define K_VOL_DOWN (0x01<<4) //right
#define F8      0x41 //right
#define F9      0x42 //left
#define F10     0x43 //down
#define F11     0x44 //up
#define F12     0x45 //right
bool sensor_ok_flag=false;
bool gyro_move = true;
bool mode_will_test=false;
bool mode_will_cal=false;
/* TWI instance. */
/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
bool Mode_2D =false;
bool Mode_3D =false;
bool Mode_test =false;
bool Mode_calibrate =false;
bool Mode_mouse =false;
bool Mode_custom1 =false;//mod_vol
//------
bool mouse_push =false;
bool push_button =false;
static int8_t send_data_t[13];
//#define touch_sum send_data_t
static int8_t key_sum[4];
static int8_t touch_sum[4];

enum data_format{
NON_DATA=0x00,
GYRO_DATA=0x01,
GSENSOR_DATA,
KEY_DATA,
TOUCH_MOVE,
DATA_3DOF=0x08,
TEST_DATA=0xFF,
};
enum Mode_select{
MODE_2D=0x10,
MODE_3D,
MODE_TEST,
MODE_CUSTOM1,
MODE_CALIBRATE,
MODE_DISCONNECT,
};
enum transfer_data{
TIME_STAMP_START,//0
TIME_STAMP_END=8,
PACKET_FLAG_START,//9
PACKET_FLAG_END=13,
DATA_MEG_X_START,//14
DATA_MEG_X_END=26,
DATA_MEG_Y_START,//27
DATA_MEG_Y_END=39,
DATA_MEG_Z_START,//40
DATA_MEG_Z_END=52,
DATA_ACC_X_START,//53
DATA_ACC_X_END=65,
DATA_ACC_Y_START,//66
DATA_ACC_Y_END=78,
DATA_ACC_Z_START,//79
DATA_ACC_Z_END=91,
DATA_GYRO_X_START,//92
DATA_GYRO_X_END=104,
DATA_GYRO_Y_START,//105
DATA_GYRO_Y_END=117,
DATA_GYRO_Z_START,//118
DATA_GYRO_Z_END=130,
TOUCH_X_START,//131
TOUCH_X_END=138,
TOUCH_Y_START,//139
TOUCH_Y_END=146,
KEY_CLICK_START,//147
KEY_CLICK_END=147,
KEY_HOME_START,//148
KEY_HOME_END=148,
KEY_APP_START,//149
KEY_APP_END=149,
KEY_VOLUME_DOWN_START,//150
KEY_VOLUME_DOWM_END=150,
KEY_VOLUME_UP_START,//151
KEY_VOLUME_UP_END=151,
MAX=160,
};
//init status
//###################################
//###################################
//###################################
//###################################
//-----------you work here -------------
//#define PROJECT_HaloMini
#ifdef PROJECT_HaloMini
static enum Mode_select MODE_INIT = MODE_2D;
static bool report_system_in_3D_mode = true;
#else
static enum Mode_select MODE_INIT = MODE_3D;   // the init mode of connection
static bool report_system_in_3D_mode = false;  // if use the function of transfer key to system in 3D mode
#endif
static bool use_mode_custom1 = false; // if use the function of control vol-+ mode
static bool use_touch_wheel = true;   // if use the function of mouse wheel
static bool open_imu_send = true;     // if use the function of read imu data and transfer it in 3D mode
//#define ONLY_TRANSFER_3DOF_DATA  //transfer data struct use every byte for units
//######################################
//######################################
//######################################
//######################################
#define SHORT_STATUS 1
#define LONG_STATUS 2
enum key_value{
KEY_UP=0,
KEY_DOWN,
KEY_LEFT,
KEY_RIGHT,
KEY_CENTRE,
KEY_BACK,
KEY_POWER,
};
//int8_t key_sum[13]={KEY_DATA,0x00};
//int8_t touch_sum[13]={TOUCH_MOVE,0x00};
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

   // err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
		err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

#define TWI_INSTANCE_ID_1     1
static const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_1);
void twi_init_1 (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_1_config = {
       .scl                = 14,//ARDUINO_SCL_PIN,
       .sda                = 13,//ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

   // err_code = nrf_drv_twi_init(&m_twi, &twi_1_config, twi_handler, NULL);
		err_code = nrf_drv_twi_init(&m_twi1, &twi_1_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi1);
}

typedef struct _Touch_format{
uint8_t Touch_Category;
uint8_t Touch_Status;
uint8_t Finger_ID;
uint16_t X_Axis;
uint16_t Y_Axis;
uint16_t X_Axis_Second;
uint16_t Y_Axis_Second;
uint8_t Z_Pressure;
uint8_t Touch_Radius;

}Touch_Event,*PTouch_Event;
Touch_Event Touch_Info;
#define PACK_INFO    0x0210
#define PACK_CONTENT 0x0211

#define TOUCH_RST_PIN 11
#define TOUCH_INT_PIN 12
#define SENSOR_INT_PIN 15
#define TOUCH_ADDRESS 0x48

uint8_t sample_data[16];
int I2C_Read_Addr8(	const uint8_t slave_addr,const uint8_t *read_addr,uint8_t addr_len,uint8_t *data,uint8_t data_num)
{
		ret_code_t err_code;
		
	  err_code = nrf_drv_twi_tx(&m_twi, slave_addr, read_addr, addr_len, false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
		err_code = nrf_drv_twi_rx(&m_twi, slave_addr, data, data_num);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	return err_code;
}
int I2C_Read_Addr(	const uint8_t slave_addr,uint8_t *data,uint8_t data_num)
{
		ret_code_t err_code;
		err_code = nrf_drv_twi_rx(&m_twi, slave_addr, data, data_num);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	return err_code;
}


int I2C_Write_Addr(	const uint8_t slave_addr,uint8_t * data,uint8_t data_num)
{
		ret_code_t err_code;
	  err_code = nrf_drv_twi_tx(&m_twi, slave_addr, data, data_num, false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return err_code;
}



static void Mode_switch(enum Mode_select Mode,bool use_mouse)
{/*
disconnect
	--test 
	--calibrate
connect 
	--test
	--calibrate
	--2d
		--mouse
		--key
	--3d
		--mouse
		--key
Mode_2d--mouse or key
Mode_3d--mouse or key
Mode_test
Mode_calibrate
*/
		Mode_2D = false;
		Mode_3D = false;
		Mode_test = false;
		Mode_custom1 = false;
		Mode_calibrate = false;
		Mode_mouse = false;
		if(Mode == MODE_2D){
			Mode_2D = true;
			Mode_mouse = use_mouse;
			if(Mode_mouse){
				sw3153_light_select(BLUE_GREEN, BLINK_LEVEL_NON);
			}else{
				sw3153_light_select(BLUE, BLINK_LEVEL_NON);
			}
		}else if(Mode == MODE_3D){
			Mode_3D = true;
			Mode_mouse = use_mouse;
			if(Mode_mouse){
			//	sw3153_blue_green();
			}else{
				sw3153_light_select(GREEN, BLINK_LEVEL_NON);
			}
		}else if(Mode == MODE_TEST){
			Mode_test = true;
			sw3153_light_select(BLUE_GREEN_RED, BLINK_LEVEL_0);
		}else if(Mode == MODE_CALIBRATE){
			Mode_calibrate = true;
		}else if(Mode == MODE_DISCONNECT){
			sw3153_light_select(BLUE, BLINK_LEVEL_2);
		}else if(Mode == MODE_CUSTOM1){
			Mode_custom1=true;
			sw3153_light_select(RED, BLINK_LEVEL_0);
		}

		if(Mode == MODE_3D || Mode == MODE_TEST || Mode == MODE_CALIBRATE || Mode_mouse == true || Mode == MODE_CUSTOM1){
			bmi160_resume();
		}else{
			bmi160_suspend();
		}

    	NRF_LOG_INFO("Mode switch Mode[%d] use_mouse[%d]\r\n",Mode,use_mouse);
}

static void devices_suspend()
{
//BMI160 IC
	bmi160_suspend();
//WD3153 IC
	sw3153_light_select(OFF, BLINK_LEVEL_NON);
//TOUCH IC
	nrf_gpio_pin_write(TOUCH_RST_PIN,0);
}
#endif  //end USE_SHADOW_CREATE
/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter_power(void)
{
    ret_code_t err_code;

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);
    // Prepare wakeup buttons.
	devices_suspend();
    NRF_LOG_INFO("power Enter sleep mode ........\r\n");
    err_code = bsp_btn_ble_sleep_mode_prepare();
    NRF_LOG_INFO("Enter sleep mode1 ........%d\r\n",err_code);
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    NRF_LOG_INFO("Enter sleep mode2 ........%d\r\n",err_code);
    //APP_ERROR_CHECK(err_code);
}

//--flash oper start
static volatile uint8_t write_flag=0;
static uint8_t cal_status=0x00;
static void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
	//NRF_LOG_INFO("fds_event_handler = 0x%x\r\n",p_fds_evt->id);
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != FDS_SUCCESS)
            {
                // Initialization failed.
            }
            break;
		case FDS_EVT_WRITE:
			if (p_fds_evt->result == FDS_SUCCESS)
			{
				write_flag=1;
				if(cal_status==0x01){
					NRF_LOG_INFO("sensor cal ok ,into sleep ....\r\n");
					sleep_mode_enter_power();
				}
			}
			break;
        default:
            break;
    }
}
static ret_code_t fds_test_write(uint32_t cal_number,uint32_t cal_data)
{
		#define FILE_ID     0x1111
		#define REC_KEY     0x2222
		static uint32_t m_deadbeef[2] = {0xDEADBEEF,0xBAADF00D};
		m_deadbeef[0] = cal_number;
		m_deadbeef[1] = cal_data;
		fds_record_t        record;
		fds_record_desc_t   record_desc;
		fds_record_chunk_t  record_chunk;
		// Set up data.
		record_chunk.p_data         = m_deadbeef;
		record_chunk.length_words   = 2;
		// Set up record.
		record.file_id              = FILE_ID;
		record.key              		= REC_KEY;
		record.data.p_chunks       = &record_chunk;
		record.data.num_chunks   = 1;
				
		ret_code_t ret = fds_record_write(&record_desc, &record);
		if (ret != FDS_SUCCESS)
		{
				return ret;
		}
		 NRF_LOG_INFO("Writing Record ID = %d \r\n",record_desc.record_id);
		return NRF_SUCCESS;
}
static ret_code_t fds_read(uint32_t *gryo_offset)
{
		#define FILE_ID     0x1111
		#define REC_KEY     0x2222
		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint32_t *data;
		uint32_t err_code;
		gryo_offset[0]=0x00000000;
		gryo_offset[1]=0x00000000;
		NRF_LOG_INFO("Start searching... \r\n");
		// Loop until all records with the given key and file ID have been found.
		while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
		{
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != FDS_SUCCESS)
				{
					return err_code;		
				}
				
				NRF_LOG_INFO("Found Record ID = %d\r\n",record_desc.record_id);
				NRF_LOG_INFO("Data = ");
				data = (uint32_t *) flash_record.p_data;
				for (uint8_t i=0;i<flash_record.p_header->tl.length_words;i++)
				{
					NRF_LOG_INFO("0x%8x ",data[i]);
				}
				gryo_offset[0]=data[0];
				gryo_offset[1]=data[1];
				NRF_LOG_INFO("\r\n");
				// Access the record through the flash_record structure.
				// Close the record when done.
				err_code = fds_record_close(&record_desc);
				if (err_code != FDS_SUCCESS)
				{
					return err_code;	
				}
		}
		return NRF_SUCCESS;
		
}

static ret_code_t fds_test_find_and_delete (void)
{
	#define FILE_ID     0x1111
	#define REC_KEY     0x2222
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
	
		ftok.page=0;
		ftok.p_addr=NULL;
		// Loop and find records with same ID and rec key and mark them as deleted. 
		while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
		{
			fds_record_delete(&record_desc);
			NRF_LOG_INFO("Deleted record ID: %d \r\n",record_desc.record_id);
		}
		// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret_code_t ret = fds_gc();
		if (ret != FDS_SUCCESS)
		{
				return ret;
		}
		return NRF_SUCCESS;
}

static ret_code_t fds_test_init (void)
{
	
		ret_code_t ret = fds_register(my_fds_evt_handler);
		if (ret != FDS_SUCCESS)
		{
					return ret;
				
		}
		ret = fds_init();
		if (ret != FDS_SUCCESS)
		{
				return ret;
		}
		
		return NRF_SUCCESS;
		
}
uint32_t gyro_offset_h=0x00000000;
uint32_t gyro_offset_l=0x00000000;
short apply_gyro_offset_X;
short apply_gyro_offset_Y;
short apply_gyro_offset_Z;
short apply_gsensor_offset_X;
short apply_gsensor_offset_Y;
short apply_gsensor_offset_Z;
void bmi160_calibration(void)
{
		//flash

	//APP_ERROR_CHECK(err_code);
	uint32_t read_gryo_offset[2]={0x00};
	fds_test_init();
	fds_read(read_gryo_offset);
	if((read_gryo_offset[1] & 0xFFFF0000)){
		sw3153_light_select(RED, BLINK_LEVEL_NON);
		nrf_delay_ms(2000);
		if(! bsp_button_is_pressed(1/*back*/)){
    		NRF_LOG_INFO("haved apply offset not apply this time\r\n");
			sleep_mode_enter_power();
			return;
		}
	}
	short cal_offset[6]={0x00};

	apply_gyro_offset_X=0x0;
	apply_gyro_offset_Y=0x0;
	apply_gyro_offset_Z=0x0;
	apply_gsensor_offset_X=0x0;
	apply_gsensor_offset_Y=0x0;
	apply_gsensor_offset_Z=0x0;
	gyro_offset_h=0x00000000;
	gyro_offset_l=0x00000000;
	cal_status=0x00;
	
    NRF_LOG_INFO("sensor calibrate start !!! \r\n");
	sw3153_light_select(RED, BLINK_LEVEL_0);
	nrf_delay_ms(3000);
	sw3153_light_select(BLUE_GREEN, BLINK_LEVEL_NON);
	nrf_delay_ms(3000);
	sw3153_light_select(BLUE_GREEN, BLINK_LEVEL_0);
	for(char i=0;i<100;i++){
		short cal_buf[6]={0x00};
		SENSOR_READ_TEST_3(cal_buf);
    	NRF_LOG_INFO("gyro offset [%d][%d][%d]\r\n",cal_buf[0],cal_buf[1],cal_buf[2]);
		cal_offset[0] += cal_buf[0];
		cal_offset[1] += cal_buf[1];
		cal_offset[2] += cal_buf[2];
		cal_offset[3] += cal_buf[3];
		cal_offset[4] += cal_buf[4];
		cal_offset[5] += cal_buf[5];
		nrf_delay_ms(20);
	}
    	NRF_LOG_INFO("gyro offset 100 sum[%d][%d][%d]\r\n",cal_offset[0],cal_offset[1],cal_offset[2]);
		cal_offset[0] = cal_offset[0]/100;
		cal_offset[1] = cal_offset[1]/100;
		cal_offset[2] = cal_offset[2]/100;
    	NRF_LOG_INFO("gyro offset 100 avg [%d][%d][%d]\r\n",cal_offset[0],cal_offset[1],cal_offset[2]);
		
	if(abs(cal_offset[0])>100 || abs(cal_offset[1])>100 || abs(cal_offset[2]>100)){
		cal_offset[0] = cal_offset[1] = cal_offset[2] = 50;
		cal_status=0x00;
		Mode_switch(MODE_DISCONNECT,false);
		sw3153_light_select(RED, BLINK_LEVEL_0);
		nrf_delay_ms(2000);
    	NRF_LOG_INFO("sensor calibrate fail ,into sleep !!! \r\n");
		sleep_mode_enter_power();
	}else{
		cal_status=0x01;
		gyro_offset_l = ((0xFFFF0000)&(cal_offset[1]<<16)) | ((0x0000FFFF)&cal_offset[0]);
		gyro_offset_h = ((0xFFFF0000)&cal_status<<16)      | ((0x0000FFFF)&cal_offset[2]);
    	NRF_LOG_INFO("gyro offset h[0x%x]--l[0x%x]\r\n",gyro_offset_h,gyro_offset_l);
//flash oper
		uint32_t err_code;
		err_code =fds_test_init();
		//APP_ERROR_CHECK(err_code);
		err_code = fds_test_find_and_delete();
		err_code = fds_test_write(gyro_offset_l,gyro_offset_h);
		//nrf_delay_ms(2000);
		Mode_switch(MODE_DISCONNECT,false);
		sw3153_light_select(BLUE_GREEN, BLINK_LEVEL_2);
		//sleep_mode_enter_power();
	}

    NRF_LOG_INFO("sensor calibrate end !!! \r\n");

	//APP_ERROR_CHECK(err_code);
	//flash
		//wait until the write is finished. 
		//while (write_flag==0);
	//fds_read();		
}


void bmi160_cal_offset_apply()
{
	uint32_t err_code;
	uint32_t read_gryo_offset[2]={0x00};
	err_code =fds_test_init();

	fds_read(read_gryo_offset);
	if(read_gryo_offset[1] & 0xFFFF0000){
		apply_gyro_offset_X = read_gryo_offset[0] & 0x0000FFFF;
		apply_gyro_offset_Y = (read_gryo_offset[0] & 0xFFFF0000) >> 16;
		apply_gyro_offset_Z = read_gryo_offset[1] & 0x0000FFFF;
    	NRF_LOG_INFO("gyro offset read ok !!  h[0x%x]  l[0x%x]\r\n",read_gryo_offset[1],read_gryo_offset[0]);
		if(abs(apply_gyro_offset_X) >100 ||abs(apply_gyro_offset_Y) >100 || abs(apply_gyro_offset_Z) >100){
			apply_gyro_offset_X =0;
			apply_gyro_offset_Y =0;
			apply_gyro_offset_Z =0;
    		NRF_LOG_INFO("gyro offset to big ,not apply set x[0]  y[0] z[0] ----- fail !!!\r\n");
		}else{
    		NRF_LOG_INFO("gyro offset apply  x[%d]  y[%d] z[%d]\r\n",apply_gyro_offset_X,apply_gyro_offset_Y,apply_gyro_offset_Z);
		}
	}else{
    	NRF_LOG_INFO("gyro offset read fail !!  h[0x%x]  l[0x%x]\r\n",read_gryo_offset[1],read_gryo_offset[0]);
	}

}
//--flash oper end
static void connect_sleep_handler(void* p_context)
{
	NRF_LOG_INFO("no operation to long goto sleep now ....\r\n");
	sleep_mode_enter_power();
}
static void connect_sleep_init(void)
{
	app_timer_create(&connect_sleep_id,APP_TIMER_MODE_SINGLE_SHOT,connect_sleep_handler);
}
static void connect_sleep_start(void)
{
	app_timer_start(connect_sleep_id,APP_TIMER_TICKS(60000*8),NULL);//8min
}
static void connect_sleep_stop(void)
{
	app_timer_stop(connect_sleep_id);
}
//--saadc oper start
extern int saadc_sample_time_ms;
static void saadc_sample_start(void)
{
	app_timer_start(saadc_sample_id,APP_TIMER_TICKS(saadc_sample_time_ms),NULL);
}
static void saadc_sample_stop(void)
{
	app_timer_stop(saadc_sample_id);
}
static void saadc_sample_handler(void* p_context)
{
	//NRF_LOG_INFO("saadc_sample_handler -------------\r\n");
  nrf_drv_saadc_sample();
	saadc_sample_start();
}
static void saadc_sample_init(void)
{
	NRF_LOG_INFO("saadc_sample_init -------------\r\n");
	app_timer_create(&saadc_sample_id,APP_TIMER_MODE_SINGLE_SHOT,saadc_sample_handler);
}


#define MTU 20
//int8_t sensor_data[3*(sizeof(float)*3+1)]={0};
uint8_t sensor_data[MTU]={0};
static uint32_t custom_char_add(ble_bas_t * p_bas, const ble_bas_init_t * p_bas_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	
	memset(&cccd_md,0,sizeof(cccd_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&attr_md,0,sizeof(attr_md));	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&attr_char_value,0,sizeof(attr_char_value));
	BLE_UUID_BLE_ASSIGN(ble_uuid, 0x1524);
	//ble_uuid.type = p_bas->uuid_type;
	//ble_uuid.uuid = 0x1524 ;
	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = sizeof(sensor_data);
	attr_char_value.max_len = sizeof(sensor_data);
	attr_char_value.init_offs = 0;
	attr_char_value.p_value = sensor_data;

	memset(&char_md,0,sizeof(char_md));
	char_md.char_props.read = 1;
	char_md.char_props.write = 1;
	char_md.char_props.notify = 1;
	char_md.p_cccd_md       = &cccd_md;
	return sd_ble_gatts_characteristic_add(p_bas->service_handle,&char_md,&attr_char_value,&custom_char_handles);

}
/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
/**@brief Event structure for @ref BLE_GATTS_EVT_WRITE. */

void custom_on_write(ble_bas_t * p_bas, ble_evt_t * p_ble_evt)
{
    //if (p_bas->is_notification_supported)
    {
        ble_gatts_evt_write_t* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    	NRF_LOG_INFO("p_evt_write->len [%d]\r\n",p_evt_write->len);
        if ((p_evt_write->handle == custom_char_handles.value_handle))
        {
					for(int i =0;i<p_evt_write->len;i++)
							NRF_LOG_INFO("p_evt_write->data [%c]\r\n",p_evt_write->data[i]);
			if(p_evt_write->data[0] == '3'){//'3'
				//Mode_3D = true;
				//Mode_mouse = false;
				//sw3153_green();
				//Mode_switch(true,false,false);
				Mode_switch(MODE_3D,false);
			}else if(p_evt_write->data[0] == '2'){//'2'
				//Mode_3D = false;
				Mode_switch(MODE_2D,false);
			}else if(p_evt_write->data[0] == 'T'){
				//Mode_switch(false,false,true);
				Mode_switch(MODE_TEST,false);
			}else if(p_evt_write->data[0] == 'M'){
				//Mode_switch(false,true,false);
				Mode_switch(MODE_2D,true);
			}else if(p_evt_write->data[0] == 'C'){
				Mode_switch(MODE_CALIBRATE,false);
			}else if(p_evt_write->data[0] == 'R'){	
				sw3153_light_select(RED, BLINK_LEVEL_NON);
			}else if(p_evt_write->data[0] == 'G'){
				sw3153_light_select(BLUE_GREEN, BLINK_LEVEL_NON);
			}else if(p_evt_write->data[0] == 'B'){	
				sw3153_light_select(BLUE, BLINK_LEVEL_NON);
			}else if(p_evt_write->data[0] == 'I'){
				open_imu_send = true;
			}else if(p_evt_write->data[0] == 'i'){
				open_imu_send = false;
			}else if(p_evt_write->data[0] == 'W'){
				use_touch_wheel = true;
			}else if(p_evt_write->data[0] == 'w'){
				use_touch_wheel = false;
			}
        }
    }
}
enum touch_action{
NOTHING,
MOVE_UP_SLOW_0,
MOVE_UP__SLOW_1,
MOVE_UP_FAST_0,
MOVE_UP_FAST_1,
MOVE_DOWN_SLOW_0,
MOVE_DOWN__SLOW_1,
MOVE_DOWN_FAST_0,
MOVE_DOWN_FAST_1,
};
#define ONE_STEP 25
static uint32_t touch_action_detect_flag=0;
static Touch_Event Touch_Info_record;
static enum touch_action action=NOTHING;
static int16_t touch_buffer[5];
static int16_t step_Y,step_temp_Y;
static int16_t step_X,step_temp_X;
static void touch_action_detect_start(void)
{
	if(use_touch_wheel == true){
		app_timer_start(touch_action_detect_id,APP_TIMER_TICKS(10),NULL);
	}
}
static void touch_action_detect_stop(void)
{

	app_timer_stop(touch_action_detect_id);
    	//NRF_LOG_INFO("----- ********************** reset11\r\n");
	if(Touch_Info.Touch_Status == 0){
		touch_action_detect_flag=0;
		step_temp_Y=0;
		step_Y=0;
		step_temp_X=0;
		step_X=0;
	}


}

static bool ble_gatts_evt_hvn_tx_complete=false;
//float buf_need_resent[6]={0.0};
bool gyro_resent_flag=false;
uint32_t custom_on_send(uint16_t conn_handle,ble_bas_t * p_bas,int8_t *send_data,uint16_t data_len)
{
    //if (p_bas->is_notification_supported)
	
    uint32_t err_code=0;

	if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
		ble_gatts_hvx_params_t params;

		memset(&params,0,sizeof(params));
		params.type = BLE_GATT_HVX_NOTIFICATION;
		params.handle = custom_char_handles.value_handle;
		params.p_data = send_data;
		params.p_len = &data_len;

		err_code = sd_ble_gatts_hvx(conn_handle,&params);
		static int i=0;
		if(send_data[0] == KEY_DATA || send_data[0] == TOUCH_MOVE){
			if(err_code == 0){NRF_LOG_INFO("----- ----------------------key/touch 0-2:[0x%x][%d][%d] err:[%d] %d\r\n",send_data[0],send_data[1],send_data[2],err_code,i++);}
			else 			 {NRF_LOG_INFO("----- -------ERR !!! -------key/touch 0-2:[0x%x][%d][%d] err:[%d] %d\r\n",send_data[0],send_data[1],send_data[2],err_code,i++);}
		}
		static int j=0;
		j++;
		if(err_code != 0 && (send_data[13]==GYRO_DATA)){
			gyro_resent_flag=true;
		    static int i=0,t;
    		NRF_LOG_INFO("----- ---------------------- gyro_data error %d [%d]ms [%d]\r\n",err_code,(j-i)*10,t++);
			i=j;
		}else if(err_code == 0 && (send_data[0]==GYRO_DATA)){
			gyro_resent_flag=false;
		}
		if(err_code != 0 && ((send_data[0] == KEY_DATA)||(send_data[0] == TOUCH_MOVE))){
		//if(err_code == 19 && ((send_data[0] == KEY_DATA))){
		//if(err_code == 19 && ((send_data[0] == KEY_DATA))){
			if(send_data[0]==TOUCH_MOVE){
				touch_action_detect_stop();
			}
    		//NRF_LOG_INFO("----- ---------------------- key/touch[0x%x] error %d %d\r\n",send_data[0],err_code,i++);
			nrf_delay_ms(50);
			err_code = sd_ble_gatts_hvx(conn_handle,&params);
			if(err_code == 19){
    			NRF_LOG_INFO("----- ---------------------- error_code resend %d error\r\n",err_code);
			}else{
    			NRF_LOG_INFO("----- ---------------------- error_code resend %d ok\r\n",err_code);
			}
			if(send_data[0]==TOUCH_MOVE){
				touch_action_detect_start();
				step_Y=step_temp_Y = (Touch_Info.Y_Axis-Touch_Info.Y_Axis_Second) / ONE_STEP;
				step_X=step_temp_X = (Touch_Info.X_Axis-Touch_Info.X_Axis_Second) / ONE_STEP;
				if(Touch_Info.Touch_Status == 0){
					touch_action_detect_flag=0;
					step_temp_Y=0;
					step_Y=0;
					step_temp_X=0;
					step_X=0;
				}
			}
		}
    }
	return err_code;
}


static void touch_action_detect_handler(void* p_context)
{
	touch_action_detect_flag++;
	//NRF_LOG_INFO("touch interrupt hander--------------%d\r\n",touch_action_detect_flag);
//#define TOUCH_SPEED
#ifdef TOUCH_SPEED
	if(touch_action_detect_flag == 1){
		Touch_Info_record.X_Axis = 	Touch_Info.X_Axis_Second;
		Touch_Info_record.Y_Axis = 	Touch_Info.Y_Axis_Second;
		for(int i=0;i<5;i++){
			touch_buffer[i]=0;
		}
	}
#define DETECTION_TIME 5
#define MIN_SPEED 5
	if(touch_action_detect_flag % DETECTION_TIME==0){
		if(abs(Touch_Info_record.Y_Axis-Touch_Info.Y_Axis_Second) > MIN_SPEED){
			
#ifdef MOVE_BUFFER
			static char i=0;
			touch_buffer[4]=0;
			touch_buffer[i++]=Touch_Info_record.Y_Axis-Touch_Info.Y_Axis_Second;
			if(i>=4)i=0;
			for(int j=0;j<4;j++){
				touch_buffer[4]+=touch_buffer[j];
			}
			touch_buffer[4]=touch_buffer[4]/4;
#else
			touch_buffer[4] = Touch_Info_record.Y_Axis-Touch_Info.Y_Axis_Second;
#endif
			//NRF_LOG_INFO("touch action ********** detY[%d] [%d][%d][%d][%d]\r\n",touch_buffer[4],touch_buffer[0],touch_buffer[1],touch_buffer[2],touch_buffer[3]);
			touch_sum[0] = TOUCH_MOVE;	
			touch_sum[1] = (touch_buffer[4]);
			//custom_on_send(m_conn_handle,&m_bas,touch_sum,13);

		}
		Touch_Info_record.X_Axis = 	Touch_Info.X_Axis_Second;
		Touch_Info_record.Y_Axis = 	Touch_Info.Y_Axis_Second;
	}
#else

	if(touch_action_detect_flag % 1==0 && Touch_Info.Touch_Status == 1 ){
		//NRF_LOG_INFO("touch action 100--------------detY[%d]\r\n",(Touch_Info.Y_Axis-Touch_Info.Y_Axis_Second)/ONE_STEP);
		
			
		step_temp_Y = (Touch_Info.Y_Axis-Touch_Info.Y_Axis_Second) / ONE_STEP;
		step_temp_X = (Touch_Info.X_Axis-Touch_Info.X_Axis_Second) / ONE_STEP;
		//NRF_LOG_INFO("touch action 100--------------step_Y[%d] step_temp_Y[%d] [%d] [%d][%d]\r\n",step_Y,step_temp_Y,step_temp_Y-step_Y,Touch_Info.Y_Axis,Touch_Info.Y_Axis_Second);
		if(step_temp_Y != step_Y){
			//NRF_LOG_INFO("touch action 100--------------detY[%d] [%d][%d] [%d][%d]\r\n",step_temp_Y-step_Y,step_temp_Y,step_Y,Touch_Info.Y_Axis,Touch_Info.Y_Axis_Second);
			//NRF_LOG_INFO("touch action 100--------------detY[%d]\r\n",step_temp_Y-step_Y);
			if(step_temp_Y-step_Y>0){//up
					touch_sum[0] = TOUCH_MOVE;	
					touch_sum[1] = (0x01);
					//custom_on_send(m_conn_handle,&m_bas,touch_sum,13);
			}else if(step_temp_Y-step_Y<0){//down
					touch_sum[0] = TOUCH_MOVE;	
					touch_sum[1] = (0x02);
					//custom_on_send(m_conn_handle,&m_bas,touch_sum,13);
			}
			step_Y=step_temp_Y;
		}
#if 0
		if(step_temp_X != step_X){
			//NRF_LOG_INFO("touch action 100--------------detX[%d] [%d][%d] [%d][%d]\r\n",step_temp_X-step_X,step_temp_X,step_X,Touch_Info.Y_Axis,Touch_Info.Y_Axis_Second);
			NRF_LOG_INFO("touch action 100--------------detX[%d]\r\n",step_temp_X-step_X);
			if(step_temp_X-step_X>0){//
					touch_sum[1] = (0x03);
					custom_on_send(m_conn_handle,&m_bas,touch_sum,13);
					report_key_touch = K_RIGHT;
					ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,&report_key_touch);
					report_key_touch = 0x00;
					ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,&report_key_touch);
			}else if(step_temp_X-step_X<0){//down
					touch_sum[1] = (0x04);
					custom_on_send(m_conn_handle,&m_bas,touch_sum,13);
					report_key_touch = K_LEFT;
					ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,&report_key_touch);
					report_key_touch = 0x00;
					ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,&report_key_touch);

			}
			step_X=step_temp_X;
		}
#endif
		
	}

#endif
}
static void touch_atcion_detect_init(void)
{
	//app_timer_create(&touch_action_detect_id,APP_TIMER_MODE_SINGLE_SHOT,touch_action_detect_handler);
	app_timer_create(&touch_action_detect_id,APP_TIMER_MODE_REPEATED,touch_action_detect_handler);
}



/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
 	err_code = custom_char_add(&m_bas,&bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                err_code;
    ble_hids_init_t           hids_init_obj;
    ble_hids_inp_rep_init_t   inp_rep_array[INPUT_REPORT_COUNT];
    ble_hids_inp_rep_init_t * p_input_report;
    uint8_t                   hid_info_flags;

    static uint8_t rep_map_data[] =
    {
        0x05, 0x01, // Usage Page (Generic Desktop)
        0x09, 0x02, // Usage (Mouse)

        0xA1, 0x01, // Collection (Application)

        // Report ID 1: Mouse buttons + scroll/pan
        0x85, 0x01,       // Report Id 1
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x95, 0x05,       // Report Count (3)
        0x75, 0x01,       // Report Size (1)
        0x05, 0x09,       // Usage Page (Buttons)
        0x19, 0x01,       // Usage Minimum (01)
        0x29, 0x05,       // Usage Maximum (05)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x01,       // Logical Maximum (1)
        0x81, 0x02,       // Input (Data, Variable, Absolute)
        0x95, 0x01,       // Report Count (1)
        0x75, 0x03,       // Report Size (3)
        0x81, 0x01,       // Input (Constant) for padding
        0x75, 0x08,       // Report Size (8)
        0x95, 0x01,       // Report Count (1)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x38,       // Usage (Wheel)
        0x15, 0x81,       // Logical Minimum (-127)
        0x25, 0x7F,       // Logical Maximum (127)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0x05, 0x0C,       // Usage Page (Consumer)
        0x0A, 0x38, 0x02, // Usage (AC Pan)
        0x95, 0x01,       // Report Count (1)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0,             // End Collection (Physical)

        // Report ID 2: Mouse motion
        0x85, 0x02,       // Report Id 2
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x75, 0x0C,       // Report Size (12)
        0x95, 0x02,       // Report Count (2)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x30,       // Usage (X)
        0x09, 0x31,       // Usage (Y)
        0x16, 0x01, 0xF8, // Logical maximum (2047)
        0x26, 0xFF, 0x07, // Logical minimum (-2047)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0xC0,             // End Collection (Physical)
        0xC0,             // End Collection (Application)

        // Report ID 3: Advanced buttons
        0x05, 0x0C,       // Usage Page (Consumer)
        0x09, 0x01,       // Usage (Consumer Control)
        0xA1, 0x01,       // Collection (Application)
        0x85, 0x03,       // Report Id (3)
        0x15, 0x00,       // Logical minimum (0)
        0x25, 0x01,       // Logical maximum (1)
        0x75, 0x01,       // Report Size (1)
        0x95, 0x01,       // Report Count (1)

        0x09, 0xCD,       // Usage (Play/Pause)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB5,       // Usage (Scan Next Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB6,       // Usage (Scan Previous Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

        0x09, 0xEA,       // Usage (Volume Down)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xE9,       // Usage (Volume Up)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x25, 0x02, // Usage (AC Forward)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x24, 0x02, // Usage (AC Back)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0,              // End Collection
				
        // Report ID 4: Advanced buttons
        0x85, 0x04,       // Report Id (4)		
        0x05, 0x01,       // Usage Page (Generic Destop)
        0x09, 0x06,       // Usage (Keyboard)
        0xA1, 0x01,       // Collection (Application)
        0x95, 0x01,       // Report Count (1)
        0x75, 0x08,       // Report Size (8)
        0x15, 0x00,       // Logical minimum (01)
        0x25, 0xFF,       // Logical maximum (255)
				0x05, 0x07,       // Usage Page (key Codes)
        0x19, 0x00,       // Usage Minimum (0)
        0x29, 0x65,       // Usage Maximum (101)
        0x81, 0x00,       // Input (Data,Variable,Absoulte)
        0xC0,              // End Collection
/*				
        0xA1, 0x01, // Collection (Application)
        0x05, 0x0C,       // Usage Page (Consumer)
        0x09, 0x01,       // Usage (Consumer Control)
        0xA1, 0x01,       // Collection (Application)
        0x85, 0x05,       // Report Id (5)
		0x19, 0x24,0x02,
		0x29, 0x24,0x02,
        0x15, 0x00,       // Logical minimum (0)
        0x25, 0x01,       // Logical maximum (1)
        0x75, 0x01,       // Report Size (8)
        0x95, 0x01,       // Report Count (1)
        0x0A, 0x24, 0x02, // Usage (AC Back)
        0x81, 0x02,       // Input (Data,Value,Relative,Bit Field)
        0xC0,              // End Collection
        0xC0,             // End Collection (Application)	
*/

        0x85, 0x05,       // Report Id (5)
        0x05, 0x0C,       // Usage Page (Consumer)
        0x09, 0x01,       // Usage (Consumer Control)
        0xA1, 0x01,       // Collection (Application)
				0x19, 0x24,0x02,
				0x29, 0x24,0x02,
        0x15, 0x00,       // Logical minimum (0)
        0x25, 0x01,       // Logical maximum (1)
        0x75, 0x01,       // Report Size (8)
        0x95, 0x01,       // Report Count (1)
        0x0A, 0x24, 0x02, // Usage (AC Back)
        0x81, 0x02,       // Input (Data,Value,Relative,Bit Field)
        0xC0,             // End Collection (Application)	
    };

    memset(inp_rep_array, 0, sizeof(inp_rep_array));
    // Initialize HID Service.
    p_input_report                      = &inp_rep_array[INPUT_REP_BUTTONS_INDEX];
    p_input_report->max_len             = INPUT_REP_BUTTONS_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_BUTTONS_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MOVEMENT_INDEX];
    p_input_report->max_len             = INPUT_REP_MOVEMENT_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOVEMENT_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MPLAYER_INDEX];
    p_input_report->max_len             = INPUT_REP_MEDIA_PLAYER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_CUSTOM1_INDEX];
    p_input_report->max_len             = INPUT_REP_CUSTOM1_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_CUSTOM1_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    p_input_report                      = &inp_rep_array[INPUT_REP_CUSTOM2_INDEX];
    p_input_report->max_len             = INPUT_REP_CUSTOM2_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_CUSTOM2_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
		
    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;
    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;
    hids_init_obj.is_mouse                       = true;
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array                = inp_rep_array;
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
    hids_init_obj.rep_map.p_data                 = rep_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    dis_init();
    bas_init();
    hids_init();

    uint32_t err_code;
    ble_dfu_init_t dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler                               = ble_dfu_evt_handler;
    dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
    dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
{
    ret_code_t err_code;

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);
if(Mode_test ==false){
    // Prepare wakeup buttons.
	devices_suspend();
    NRF_LOG_INFO("not test Enter sleep mode ........\r\n");
    err_code = bsp_btn_ble_sleep_mode_prepare();
    NRF_LOG_INFO("Enter sleep mode1 ........%d\r\n",err_code);
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    NRF_LOG_INFO("Enter sleep mode2 ........%d\r\n",err_code);
    //APP_ERROR_CHECK(err_code);
}
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("Directed advertising.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with whitelist.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with whitelist.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("BLE_ADV_EVT_IDLE will sleep\r\n");
            //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            //APP_ERROR_CHECK(err_code);
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist\r\n",
                           addr_cnt,
                           irk_cnt);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(whitelist_addrs, addr_cnt,
                                                       whitelist_irks,  irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {

                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            NRF_LOG_INFO("Param update ...............\r\n");
		break;
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");

			if(Mode_test == true){
				Mode_switch(MODE_TEST,false);
			}else if(Mode_calibrate == true){
				Mode_switch(MODE_CALIBRATE,false);
			}else if(Mode_3D == true){
				if(Mode_mouse == false){
					Mode_switch(MODE_3D,false);
				}else{
					Mode_switch(MODE_3D,true);
				}
			}else if(Mode_2D == true){
				if(Mode_mouse == false){
					Mode_switch(MODE_2D,false);
				}else{
					Mode_switch(MODE_2D,true);
				}
			}else{
				Mode_switch(MODE_INIT,false);
			}
			
			if(Mode_2D == true || Mode_3D == true){
            	NRF_LOG_INFO("connect into 2D or 3D ,start connect_sleep_start timer\r\n");
				connect_sleep_start();
			}
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected\r\n");

			connect_sleep_stop();

			if(Mode_test == true){
				Mode_switch(MODE_TEST,false);
			}else if(Mode_calibrate == true){
				Mode_switch(MODE_CALIBRATE,false);
			}else{
				Mode_switch(MODE_DISCONNECT,false);
			}
            //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            //APP_ERROR_CHECK(err_code);

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            if (m_is_wl_changed)
            {
                // The whitelist has been modified, update it in the Peer Manager.
                err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                APP_ERROR_CHECK(err_code);

                err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                if (err_code != NRF_ERROR_NOT_SUPPORTED)
                {
                    APP_ERROR_CHECK(err_code);
                }

                m_is_wl_changed = false;
            }
            break; // BLE_GAP_EVT_DISCONNECTED

		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            //NRF_LOG_DEBUG("GATTs evt_hvn_tx complete !!!!!!!!!!!!!!!!!!!!!!!\r\n");
			ble_gatts_evt_hvn_tx_complete = true;
			break;
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    //NRF_LOG_INFO(" ----  ble_evt_dispatch :header.evt_id = 0x%x\r\n",p_ble_evt->header.evt_id);
	if(p_ble_evt->header.evt_id == 0x57){
		if(Mode_3D != true){
			connect_sleep_stop();
			connect_sleep_start();
            //NRF_LOG_DEBUG("--------------connect sleep restart\r\n");
		}else{
			if(true == gyro_move){
				connect_sleep_stop();
				connect_sleep_start();
            	//NRF_LOG_DEBUG("-------3D-------connect sleep restart\r\n");
			}
		}
	}
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_hids_on_ble_evt(&m_hids, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);

    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    //NRF_LOG_INFO(" --------------------\r\n");
    fs_sys_event_handler(sys_evt);
    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the number of custom UUIDS.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 1;
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    uint8_t                adv_flags;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    adv_flags                       = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = adv_flags;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_whitelist_enabled      = false;
    options.ble_adv_directed_enabled       = true;
    options.ble_adv_directed_slow_enabled  = false;
    options.ble_adv_directed_slow_interval = 0;
    options.ble_adv_directed_slow_timeout  = 0;
    options.ble_adv_fast_enabled           = true;
    options.ble_adv_fast_interval          = APP_ADV_FAST_INTERVAL;
    options.ble_adv_fast_timeout           = APP_ADV_FAST_TIMEOUT;
    options.ble_adv_slow_enabled           = true;
    options.ble_adv_slow_interval          = APP_ADV_SLOW_INTERVAL;
    options.ble_adv_slow_timeout           = APP_ADV_SLOW_TIMEOUT;

    err_code = ble_advertising_init(&advdata,
                                    NULL,
                                    &options,
                                    on_adv_evt,
                                    ble_advertising_error_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void mouse_click_send(uint8_t left_right_buffer,uint8_t Scroll_Value)
{
    uint32_t err_code;
    uint8_t buffer[3];
    if (m_in_boot_mode)
        return;    

    buffer[0] = left_right_buffer; // Left button (bit 0) pressed
    buffer[1] = Scroll_Value; // Scroll value (-127, 128)
    buffer[2] = 0; // Sideways scroll value (-127, 128)
		
    err_code = ble_hids_inp_rep_send(&m_hids,INPUT_REP_BUTTONS_INDEX, INPUT_REP_BUTTONS_LEN, buffer);
    NRF_LOG_INFO("----- ---------------------- error_code %d \r\n",err_code);
	if(err_code == 19){
	
	nrf_delay_ms(50);
    err_code = ble_hids_inp_rep_send(&m_hids,INPUT_REP_BUTTONS_INDEX, INPUT_REP_BUTTONS_LEN, buffer);
    NRF_LOG_INFO("----- ---------------------- error_code resend %d \r\n",err_code);
	}

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_CHECK(err_code);
    }
}
/**@brief Function for sending a Mouse Movement.
 *
 * @param[in]   x_delta   Horizontal movement.
 * @param[in]   y_delta   Vertical movement.
 */
static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
{
    ret_code_t err_code;

    if (m_in_boot_mode)
    {
        x_delta = MIN(x_delta, 0x00ff);
        y_delta = MIN(y_delta, 0x00ff);

        err_code = ble_hids_boot_mouse_inp_rep_send(&m_hids,
                                                    0x00,
                                                    (int8_t)x_delta,
                                                    (int8_t)y_delta,
                                                    0,
                                                    NULL);
    }
    else
    {
        uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

        APP_ERROR_CHECK_BOOL(INPUT_REP_MOVEMENT_LEN == 3);

        x_delta = MIN(x_delta, 0x0fff);
        y_delta = MIN(y_delta, 0x0fff);

        buffer[0] = x_delta & 0x00ff;
        buffer[1] = ((y_delta & 0x000f) << 4) | ((x_delta & 0x0f00) >> 8);
        buffer[2] = (y_delta & 0x0ff0) >> 4;

        err_code = ble_hids_inp_rep_send(&m_hids,
                                         INPUT_REP_MOVEMENT_INDEX,
                                         INPUT_REP_MOVEMENT_LEN,
                                         buffer);
    }
	if(err_code == 19){
	
	//nrf_delay_ms(50);
    //err_code = ble_hids_inp_rep_send(&m_hids,INPUT_REP_BUTTONS_INDEX, INPUT_REP_BUTTONS_LEN, buffer);
    NRF_LOG_INFO("----- ---------------------- error_code resend %d \r\n",err_code);
	}
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        //APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
uint8_t key_num_3;
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:

            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_1: //back

			if(Mode_test == true && sensor_ok_flag==true){
				sw3153_light_select(GREEN, BLINK_LEVEL_2);
				break;
			}
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				uint8_t temp = 0xFF;//0x01<<4;
				ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM2_INDEX,INPUT_REP_CUSTOM2_LEN,&temp);
				key_sum[0]=KEY_DATA;
				key_sum[SHORT_STATUS] |= (0x01<<KEY_BACK);
            }
            break;
        case BSP_EVENT_KEY_1_RELEASE:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				if(Mode_test == true){
					break;
				}
				uint8_t temp = 0x00;//0x01<<4;
				ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM2_INDEX,INPUT_REP_CUSTOM2_LEN,&temp);
				key_sum[0]=KEY_DATA;
				key_sum[SHORT_STATUS] &= (~(0x01<<KEY_BACK));
				key_sum[LONG_STATUS] &= (~(0x01<<KEY_BACK));
            }
            break;
        case BSP_EVENT_KEY_1_LONG:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				key_sum[0]=KEY_DATA;
				key_sum[SHORT_STATUS] |= (0x01<<KEY_BACK);
				key_sum[LONG_STATUS] |= (0x01<<KEY_BACK);
			}
            break;
        case BSP_EVENT_KEY_0://up down left rigth
			if(Mode_test == true && sensor_ok_flag==true){
				if(report_key == K_UP || report_key == F8){
					sw3153_light_select(RED, BLINK_LEVEL_0);
				}else if(report_key == K_DOWN || report_key == F9){
					sw3153_light_select(RED, BLINK_LEVEL_2);
				}else if(report_key == K_RIGHT || report_key == F11){
					sw3153_light_select(GREEN, BLINK_LEVEL_2);
				}else if(report_key == K_LEFT || report_key == F10){
					sw3153_light_select(GREEN, BLINK_LEVEL_0);
				}else if(report_key == K_ENTER || report_key == F12){
					sw3153_light_select(BLUE, BLINK_LEVEL_0);
				}
				break;
			}
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				int8_t const_enter = K_ENTER;
				if(Mode_3D == true){
					const_enter = F12;
				}
				if(Mode_mouse == false){
					if(true == report_system_in_3D_mode || Mode_3D != true){
						ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,/*&report_key*/&const_enter);
					}
					push_button = true;
				 	report_key_cache = report_key;	
					key_sum[0]=KEY_DATA;
					key_sum[SHORT_STATUS] |= (0x01 << (report_key-F8));
				}else if(Mode_mouse == true){
    				NRF_LOG_INFO("key_2 press long---------------- down\r\n");
					if(true == report_system_in_3D_mode || Mode_3D != true){
						mouse_click_send(0x01,0);
					}
				}
            }
            break;
        case BSP_EVENT_KEY_0_LONG:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				key_sum[0]=KEY_DATA;
				key_sum[SHORT_STATUS] |= (0x01 << (report_key-F8));
				key_sum[LONG_STATUS] |= (0x01 << (report_key_cache-F8));
            }
            break;
        case BSP_EVENT_KEY_0_RELEASE:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				if(Mode_test == true){
					break;
				}
				uint8_t temp = 0x00;
				if(Mode_mouse == false){
					if(true == report_system_in_3D_mode || Mode_3D != true){
						ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,&temp);
					}
					push_button = true;
					key_sum[0]=KEY_DATA;
					key_sum[SHORT_STATUS] &= (~(0x01 << (report_key_cache-F8)));
					key_sum[LONG_STATUS] &= (~(0x01 << (report_key_cache-F8)));
				}else if(Mode_mouse == true){
    				NRF_LOG_INFO("key_2 press long---------------- up\r\n");
					if(true == report_system_in_3D_mode || Mode_3D != true){
						mouse_click_send(0,0);
					}
				}
            }
            break;
        case BSP_EVENT_KEY_2:
    		NRF_LOG_INFO("Power key DOWN ---- DOWN\r\n");
			if(Mode_test == true && sensor_ok_flag==true){
				sw3153_light_select(GREEN, BLINK_LEVEL_0);
				break;
			}
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				static  enum Mode_select mode_keep;
				if(Mode_2D == true){
					mode_keep = MODE_2D;
					if(Mode_mouse == false){	
						Mode_switch(MODE_2D,true);
					}else if(Mode_mouse == true){
						if(use_mode_custom1==true){
							Mode_switch(MODE_CUSTOM1,false);
						}else{
							Mode_switch(MODE_2D,false);
						}
					}
				}else if(Mode_3D == true){
					mode_keep = MODE_3D;
					if(Mode_mouse == true){
						Mode_switch(MODE_3D,false);	
					}else{
						if(use_mode_custom1==true){
							Mode_switch(MODE_CUSTOM1,false);
						}else{
							Mode_switch(MODE_3D,false);
						}
					}
				}else if(Mode_custom1 == true && use_mode_custom1==true){
					if(mode_keep == MODE_2D){
						Mode_switch(MODE_2D,false);
					}else if(mode_keep == MODE_3D){
						Mode_switch(MODE_3D,false);
					}	
				}
            }
            break;
        case BSP_EVENT_KEY_2_RELEASE:
    		NRF_LOG_INFO("Power key UP ---- UP\r\n");
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
            }else{
			}
			break;

        case BSP_EVENT_KEY_2_LONG:
    			NRF_LOG_INFO("Power key LONG ---- LONG\r\n");
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
            }else{
			}
			sw3153_light_select(RED, BLINK_LEVEL_NON);
			nrf_delay_ms(2000);
    		NRF_LOG_INFO("power  key  push  to sleep !!! \r\n");
			sleep_mode_enter_power();
            break;
        case BSP_EVENT_KEY_3:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                //mouse_movement_send(0, MOVEMENT_SPEED);
            }
            break;

        default:
            break;
    }
}
#define MOUSE_FACTORY_X 250
#define MOUSE_FACTORY_Y 250
#define MAX_SUB_MOVE_ADD 2  //move speed
#if 0
#define SUB_ACC 0.05  //accuary
char mouse_dpi = 3;
#else
#define SUB_ACC 0.03  //accuary
char mouse_dpi = MAX_SUB_MOVE_ADD;
#endif
static float mouseX_sub,mouseY_sub;
void send_mouse_data(float *buf)
{
  float tempX,tempY,tempZ; 
  int32_t mouseX,mouseY;
#if 0
  tempX = buf[2]/*z*/ * MOUSE_FACTORY_X ;
  tempY = buf[0]/*x*/ * MOUSE_FACTORY_Y ;
	//tempZ = buf[5] * MOUSE_FACTORY_X
  mouseX = tempX / 0x8000;
  mouseY = tempY / 0x8000;
#else
//mouseX = (buf[2]*0.00763);
  buf[2]=-buf[2];
mouseX_sub +=(int) (buf[2]*SUB_ACC);//must do (int)
//mouseY = (buf[0]*0.00763);
mouseY_sub +=(int) (buf[0]*SUB_ACC);
  //NRF_LOG_INFO("-----------mouse x_sub[%d] y_sub[%d] buf0++[%d] buf2++[%d]\r\n",(int)mouseX_sub,(int)mouseY_sub,(int)buf[0]*SUB_ACC,(int)buf[2]*SUB_ACC);
 #endif
  //NRF_LOG_INFO("-----------mouse x[%d] y[%d]\r\n",(int)mouseX,(int)mouseY);
  if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
   {/*
		 if(abs(mouseX_sub) >= MAX_SUB_MOVE_ADD || abs(mouseY_sub) >= MAX_SUB_MOVE_ADD){
				mouse_movement_send((int)(mouseX_sub/MAX_SUB_MOVE_ADD), (int)(mouseY_sub/MAX_SUB_MOVE_ADD));
				if(abs(mouseX_sub)>=MAX_SUB_MOVE_ADD)
				mouseX_sub =0;
				if(abs(mouseY_sub)>=MAX_SUB_MOVE_ADD)
				mouseY_sub =0;
		 }*/
		 if(abs(mouseX_sub) >= mouse_dpi || abs(mouseY_sub) >= mouse_dpi){
				mouse_movement_send((int)(mouseX_sub/mouse_dpi), (int)(mouseY_sub/mouse_dpi));
				if(abs(mouseX_sub)>=mouse_dpi)
				mouseX_sub =0;
				if(abs(mouseY_sub)>=mouse_dpi)
				mouseY_sub =0;
		 }
		 //if((int)mouseX != 0 || (int)mouseY != 0)
		//		mouse_movement_send(-(int)mouseX, (int)mouseY);
   }
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    //err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    err_code = bsp_init( BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
	if(startup_event == BSP_EVENT_ENTER_TEST){
		mode_will_test = true;
	}else if(startup_event == BSP_EVENT_ENTER_CALIBRATE){
		mode_will_cal = true;
	}
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
    uint8_t sample_data[16];
ret_code_t eventsize;
#include "nrf_drv_gpiote.h"

uint8_t interr;
static uint16_t tp_firmware_version(void)
{
	uint16_t temp;
	uint8_t firmware[2];
	const uint8_t version_addr[2]={0x01,0x26};
	//I2C_Read_Addr16(TOUCH_ADDRESS,0x0126,firmware,2);
 I2C_Read_Addr8(TOUCH_ADDRESS,version_addr,2,firmware,2);
	temp=firmware[1];
	return (uint16_t)(temp<<8 | firmware[0]);
}

#define TOUCH_INTER_INTERVAL 10

bool touch_timer_into=false;
char touch_timer_into_=0;

static void touch_timer_handler(void* p_context)
{

touch_timer_into = true;
touch_timer_into_ ++;
						//keys_send(1, &report_key);
				//send_mouse_data(NULL);
}
static void touch_timer_init(void)
{
	app_timer_create(&touch_timer_id,APP_TIMER_MODE_SINGLE_SHOT,touch_timer_handler);
}
static void touch_timer_start(void)
{
	app_timer_start(touch_timer_id,APP_TIMER_TICKS(TOUCH_INTER_INTERVAL),NULL);
}
static void touch_timer_stop(void)
{
	app_timer_stop(touch_timer_id);
}
static void mouse_slow_handler(void* p_context)
{

	app_timer_start(sensor_poll_timer_id,APP_TIMER_TICKS(10),NULL);
}
static void mouse_slow_init(void)
{
	app_timer_create(&mouse_slow_id,APP_TIMER_MODE_SINGLE_SHOT,mouse_slow_handler);
}
static void mouse_slow_start(void)
{
	app_timer_start(mouse_slow_id,APP_TIMER_TICKS(20),NULL);
}
static void mouse_slow_stop(void)
{
	app_timer_stop(mouse_slow_id);
}

static void sensor_cal_handler(void* p_context)
{
	bmi160_calibration();
}
static void sensor_cal_init(void)
{
	app_timer_create(&sensor_cal_id,APP_TIMER_MODE_SINGLE_SHOT,sensor_cal_handler);
}
static void sensor_cal_start(void)
{
	app_timer_start(sensor_cal_id,APP_TIMER_TICKS(20),NULL);
}
static void sensor_cal_stop(void)
{
	app_timer_stop(sensor_cal_id);
}




float buf1[6]={0};
bool touch_first=false;
bool touch_first1=false;
char key_cal;
char key_cal_x;
char key_cal_y;
char iii=0;
char iiii=0;
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//NRF_LOG_INFO("sensor interrupt hander--------------\r\n");
	  ret_code_t err_code;
    static uint8_t * p_key = m_sample_key_press_scan_str;
    static uint8_t   size  = 0;
		interr++;
/*
		keys_send(1, p_key);
    p_key++;
    size++;
    if (size == MAX_KEYS_IN_ONE_REPORT)
    {
      p_key = m_sample_key_press_scan_str;
      size  = 0;
    }						
*/
				const uint8_t pack_info_addr[2]={0x02,0x10};
				I2C_Read_Addr8(TOUCH_ADDRESS,pack_info_addr,2,sample_data,1);
				eventsize = sample_data[0] & 0x7F;
				Touch_Info.Touch_Category = (sample_data[0] >> 7) & 0x01;
				if(0 == Touch_Info.Touch_Category){
						if(eventsize > 0){
								const uint8_t pack_content_addr[2]={0x02,0x11};
								I2C_Read_Addr8(TOUCH_ADDRESS,pack_content_addr,2,sample_data,eventsize);
							for(int i=0;i<eventsize/6;i++){
								Touch_Info.Finger_ID = sample_data[0+i*6] & 0x0F - 1;
								Touch_Info.Touch_Status = sample_data[0+i*6] >> 7;
								if(touch_first == false){
									touch_first = true;
									Touch_Info.X_Axis = ((sample_data[1+i*6] << 8) & 0x0F00)|sample_data[2+i*6];
									Touch_Info.Y_Axis = ((sample_data[1+i*6] << 4) & 0x0F00)|sample_data[3+i*6];
									Touch_Info.X_Axis_Second = ((sample_data[1+i*6] << 8) & 0x0F00)|sample_data[2+i*6];
									Touch_Info.Y_Axis_Second = ((sample_data[1+i*6] << 4) & 0x0F00)|sample_data[3+i*6];
								}else{
									Touch_Info.X_Axis_Second = ((sample_data[1+i*6] << 8) & 0x0F00)|sample_data[2+i*6];
									Touch_Info.Y_Axis_Second = ((sample_data[1+i*6] << 4) & 0x0F00)|sample_data[3+i*6];
								}
							}
						}					
				}
#if 0
if(Touch_Info.Touch_Status == 1){

NRF_LOG_INFO("sensor interrupt hander--------------push %d %d\r\n",Touch_Info.Finger_ID,iii++);
}else if(Touch_Info.Touch_Status == 0){

NRF_LOG_INFO("sensor interrupt hander-------------- up %d\r\n",Touch_Info.Finger_ID);
}
#endif

						/*if(-Touch_Info.X_Axis < -Touch_Info.Y_Axis && Touch_Info.X_Axis < -Touch_Info.Y_Axis ){
								report_key = K_UP; // 0x52;
						}else if(Touch_Info.X_Axis < Touch_Info.Y_Axis && -Touch_Info.X_Axis < Touch_Info.Y_Axis ){
								report_key = K_DOWN;  //0x51;
						}else if(-Touch_Info.X_Axis > -Touch_Info.Y_Axis && -Touch_Info.X_Axis > -Touch_Info.Y_Axis ){
								report_key = K_RIGHT; //0x4f
						}else if(Touch_Info.X_Axis > -Touch_Info.Y_Axis && Touch_Info.X_Axis > Touch_Info.Y_Axis ){
								report_key = K_LEFT;  //0x50
						}*/
					int x,y;
					x = 127 -Touch_Info.X_Axis_Second;
					y = 127 -Touch_Info.Y_Axis_Second;
					touch_sum[0] = TOUCH_MOVE;	
					touch_sum[2]=x;
					touch_sum[3]=y;
					if(Mode_3D == true){
						if(abs(x) < 38 && abs(y) <38){
							report_key = F12; // 0x45;
						}else if(abs(x) < (y )  ){
								report_key = F8; // 0x41;
						}else if(abs(x) <(-y )  ){
								report_key = F9;  //0x42;
						}else if(abs(y) < (x ) ){
								report_key = F11; //0x44
						}else if(abs(y) < (-x )  ){
								report_key = F10;  //0x43
						};
						//if(mouse_push == true)
						//	mouse_click_send(0,0);	
					}else{
						if(abs(x) < 38 && abs(y) <38){
							report_key = K_ENTER; // 0x28;
						}else if(abs(x) < (y )  ){
								report_key = K_UP; // 0x52;
						}else if(abs(x) <(-y )  ){
								report_key = K_DOWN;  //0x51;
						}else if(abs(y) < (x ) ){
								report_key = K_RIGHT; //0x4f
						}else if(abs(y) < (-x )  ){
								report_key = K_LEFT;  //0x50
						};
						//if(x>50)
						//	   report_mouse = 0x02;//right button
						//else if(x < -50)
						//	    report_mouse = 0x01;//left button
						//else if(x<=50 && x>=-50)
					    //		     report_mouse = 0x00;//scroll
					}

				if(Touch_Info.Touch_Status == 1 && touch_timer_into == false){
					touch_timer_start();
				}else if(Touch_Info.Touch_Status == 1 && touch_timer_into == true && touch_first1==false){
					touch_first1=true;
					if(true == Mode_3D){
						touch_action_detect_start();
					}
				}else if(Touch_Info.Touch_Status == 0){
					if(touch_timer_into == true){
						touch_action_detect_stop();
						if(abs(Touch_Info.Y_Axis_Second - Touch_Info.Y_Axis) <8 && abs(Touch_Info.X_Axis_Second - Touch_Info.X_Axis) <8  ){
								if(Mode_3D == true)report_key_touch = F12;
								else report_key_touch = K_ENTER;
    //NRF_LOG_INFO("----- ---------------------- keyyy K_ENTER \r\n");
						}else if(abs(Touch_Info.Y_Axis_Second - Touch_Info.Y_Axis) > abs(Touch_Info.X_Axis_Second - Touch_Info.X_Axis)){
							if((Touch_Info.Y_Axis_Second - Touch_Info.Y_Axis) > 18){
								if(Mode_3D == true)report_key_touch = F9;
								else report_key_touch = K_DOWN;
    //NRF_LOG_INFO("----- ---------------------- keyyy K_DOWN \r\n");
							}else if((Touch_Info.Y_Axis_Second - Touch_Info.Y_Axis) < -18){
								if(Mode_3D == true)report_key_touch = F8;
								else report_key_touch = K_UP;
    //NRF_LOG_INFO("----- ---------------------- keyyy K_UP \r\n");
							}
						}else if(abs(Touch_Info.Y_Axis_Second - Touch_Info.Y_Axis) < abs(Touch_Info.X_Axis_Second - Touch_Info.X_Axis)){
							if((Touch_Info.X_Axis_Second - Touch_Info.X_Axis) > 30){
								if(Mode_3D == true)report_key_touch = F10;
								else report_key_touch = K_LEFT;
    //NRF_LOG_INFO("----- ---------------------- keyyy K_LEFT \r\n");
							}else if((Touch_Info.X_Axis_Second - Touch_Info.X_Axis) < -30){
								if(Mode_3D == true)report_key_touch = F11;
								else report_key_touch = K_RIGHT;
    //NRF_LOG_INFO("----- ---------------------- keyyy K_RIGHT \r\n");
							}
						}else{
							//report_key_touch = 0x00;
						}
key_cal = report_key_touch;
						if(Mode_test == true){
					     //nothing	
						}else if(push_button == true){
							push_button = false;
						}else if(Mode_custom1 == true){
							if(report_key_touch == K_UP || report_key_touch == K_RIGHT){
								for(int i=0;i<2;i++){
									report_key_touch = K_VOL_UP;
									ble_hids_inp_rep_send(&m_hids,INPUT_REP_MPLAYER_INDEX,INPUT_REP_MEDIA_PLAYER_LEN,&report_key_touch);
									report_key_touch = 0x00;
									ble_hids_inp_rep_send(&m_hids,INPUT_REP_MPLAYER_INDEX,INPUT_REP_MEDIA_PLAYER_LEN,&report_key_touch);
								}
							}else if(report_key_touch == K_DOWN || report_key_touch == K_LEFT){
								for(int i=0;i<2;i++){
									report_key_touch = K_VOL_DOWN;
									ble_hids_inp_rep_send(&m_hids,INPUT_REP_MPLAYER_INDEX,INPUT_REP_MEDIA_PLAYER_LEN,&report_key_touch);
									report_key_touch = 0x00;
									ble_hids_inp_rep_send(&m_hids,INPUT_REP_MPLAYER_INDEX,INPUT_REP_MEDIA_PLAYER_LEN,&report_key_touch);
								}
							}
						}else if(report_key_touch != 0x00 && Mode_mouse == false && Mode_3D == false){
							ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,&report_key_touch);
							report_key_touch = 0x00;
							ble_hids_inp_rep_send(&m_hids,INPUT_REP_CUSTOM1_INDEX,INPUT_REP_CUSTOM1_LEN,&report_key_touch);
						}/*else if(Mode_mouse == true && mouse_push == false){
							report_mouse = 0x01;//left button
    NRF_LOG_INFO("----- ###################### 11111\r\n");
							mouse_click_send(report_mouse,0);
							mouse_push = false;
    NRF_LOG_INFO("----- ----------------------22222\r\n");
							mouse_click_send(0,0);	
						}*/

					}
					touch_timer_stop();
					touch_timer_into = false;
					touch_first = false;
					touch_first1 = false;
				}


						if(Touch_Info.Touch_Status == 1 && Mode_mouse == true && touch_timer_into == true){
							if(mouse_push == false){
								mouse_push = true;
								report_mouse = 0x01;//left button
#if 1
								mouse_dpi = MAX_SUB_MOVE_ADD+1;
#else
mouseX_sub=0;
mouseY_sub=0;
app_timer_stop(sensor_poll_timer_id);
mouse_slow_start();
#endif
								mouse_click_send(report_mouse,0);
    NRF_LOG_INFO("----- ###################### DOWN %d\r\n",iiii++);
							}
						}else if(Touch_Info.Touch_Status == 0 && Mode_mouse == true && mouse_push == true){
							mouse_push = false;
							mouse_click_send(0,0);
#if 1
							mouse_dpi=MAX_SUB_MOVE_ADD;
#endif

    NRF_LOG_INFO("----- ---------------------- UP \r\n");
						}
}

float raw_buf[3];
float dof3_buf[6];
uint16_t data1[6];
uint8_t interrupter_sensor;
int gyro_num=0;
int gsensor_num=0;
void sensor_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	#if 0
	ret_code_t err_code;
	if(Mode_mouse == true){
		//interrupter_sensor++;
		SENSOR_READ_RAW_INT(raw_buf);
		send_mouse_data(raw_buf);
		
	}
	if(Mode_3D == true){
		SENSOR_READ_TEST(dof3_buf);
		memcpy(sensor_data,(int8_t*)dof3_buf,sizeof(sensor_data)-1);
		/*
				NRF_LOG_INFO("%c-%c-%c-%c\r\n",((int8_t*)dof3_buf)[0],((int8_t*)dof3_buf)[1],((int8_t*)dof3_buf)[2],((int8_t*)dof3_buf)[3]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",((int8_t*)dof3_buf)[4],((int8_t*)dof3_buf)[5],((int8_t*)dof3_buf)[6],((int8_t*)dof3_buf)[7]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",((int8_t*)dof3_buf)[8],((int8_t*)dof3_buf)[9],((int8_t*)dof3_buf)[10],((int8_t*)dof3_buf)[11]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",((int8_t*)dof3_buf)[12],((int8_t*)dof3_buf)[13],((int8_t*)dof3_buf)[14],((int8_t*)dof3_buf)[15]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",((int8_t*)dof3_buf)[16],((int8_t*)dof3_buf)[17],((int8_t*)dof3_buf)[18],((int8_t*)dof3_buf)[19]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",((int8_t*)dof3_buf)[20],((int8_t*)dof3_buf)[21],((int8_t*)dof3_buf)[22],((int8_t*)dof3_buf)[23]);*/
		/*for(int i=0;i<sizeof(sensor_data);i++)*/
		/*
				NRF_LOG_INFO("%c-%c-%c-%c\r\n",sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",sensor_data[12],sensor_data[13],sensor_data[14],sensor_data[15]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",sensor_data[16],sensor_data[17],sensor_data[18],sensor_data[19]);
		    NRF_LOG_INFO("%c-%c-%c-%c\r\n",sensor_data[20],sensor_data[21],sensor_data[22],sensor_data[23]);
		    NRF_LOG_INFO("%c--------------\r\n",sensor_data[25]);
		*/
		/*NRF_LOG_INFO("\r\n");*/
//NRF_LOG_INFO("sensor interrupt hander--------------\r\n");
		//custom_on_send(m_conn_handle,&m_bas,sensor_data,sizeof(sensor_data));
	}
#endif
}
#define SENSOR_POLL_INTERVAL 10
void sensor_poll_start()
{
	app_timer_start(sensor_poll_timer_id,APP_TIMER_TICKS(SENSOR_POLL_INTERVAL),NULL);
}
int16_t min_param,max_param,slave_latency;
#include "nrf_drv_systick.h"
void set_bits(uint8_t start_bit,uint8_t end_bit,uint16_t value)
{
	char *Point = sensor_data;
	//*(uint32_t*)Point=0xFFFF;
	*(uint32_t*)(Point + start_bit/8) |= ((uint32_t)value)<<(start_bit%8);
	//*(uint32_t*)(Point + start_bit/8) |= ((uint32_t)value)<<(32-(end_bit%7+((end_bit-start_bit)/8)*8));
	//NRF_LOG_INFO("[%d][%d][%d][%d]",sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3]);
	//NRF_LOG_INFO("[%d][%d][%d][%d]",sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7]);
	//NRF_LOG_INFO("[%d][%d][%d][%d]",sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
	//NRF_LOG_INFO("[%d][%d][%d][%d]",sensor_data[12],sensor_data[13],sensor_data[14],sensor_data[15]);
	//NRF_LOG_INFO("[%d][%d][%d][%d]",sensor_data[16],sensor_data[17],sensor_data[18],sensor_data[19]);
	//NRF_LOG_INFO("[%x][%x][%x][%x]",(uint32_t)sensor_data,(uint32_t)(sensor_data+1),(uint32_t)(sensor_data+2),(uint32_t)Point);
	//NRF_LOG_INFO("*%d-%d* bits %d\r\n",start_bit,end_bit,(*(unsigned short*)(Point + start_bit/8)&(~(0x00<<start_bit/8)))>>(15-(end_bit-start_bit)));
	//NRF_LOG_INFO("*%d-%d* bits %d\r\n",start_bit,end_bit,((*(uint32_t*)(Point + start_bit/8)<<(start_bit%8))>>(32+(start_bit%8)-(end_bit%7+((end_bit-start_bit)/8)*8))));
	//if(start_bit >=13 && start_bit <=41 )
	//NRF_LOG_INFO("*%d-%d* value %4d\r\n",start_bit,end_bit,*(uint32_t*)(Point + start_bit/8)>>(start_bit%8));
}
void sensor_data_poll_handler(void* p_context)
{
	static nrf_drv_systick_state_t systick_s;
	static nrf_drv_systick_state_t systick_s_end;
	static nrf_drv_systick_state_t systick_s_lastest;
	static uint32_t det_tick1=0,det_tick2=0;
	nrf_drv_systick_get(&systick_s);

	app_timer_start(sensor_poll_timer_id,APP_TIMER_TICKS(SENSOR_POLL_INTERVAL),NULL);
			//nrf_delay_ms(50);
//nrf_drv_systick_delay_ms(50);
	//app_timer_stop(sensor_poll_timer_id);
	//if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
	//	return;
#if 1
	ret_code_t err_code;
	if(Mode_test == true && sensor_ok_flag == false){
		SENSOR_READ_TEST_2(dof3_buf);
		gyro_num +=(dof3_buf[0]+dof3_buf[1]+dof3_buf[2]);
		gsensor_num +=(dof3_buf[3]+dof3_buf[4]+dof3_buf[5]);
		if(abs(gyro_num)>1000 && abs(gsensor_num)>50000){
				sensor_ok_flag = true;
		}
  		NRF_LOG_INFO("----- ---------------------- TEST g[%d] s[%d] sensor_ok[%d]\r\n",gyro_num,gsensor_num,sensor_ok_flag);
	}
	if(Mode_mouse == true){
		//interrupter_sensor++;
		SENSOR_READ_RAW_INT(raw_buf);
		send_mouse_data(raw_buf);
		
	}

	if(Mode_3D == true ){
		if(open_imu_send == true){
			SENSOR_READ_TEST(dof3_buf,data1);
			if(dof3_buf[0] <= 0.2 && dof3_buf[1] <= 0.2 && dof3_buf[2] <= 0.2){
				gyro_move = false;
				//NRF_LOG_INFO("------------------------ gyro no move\n\r");
			}else{
				//NRF_LOG_INFO("------------------------ gyro  move\n\r");
				gyro_move = true;
			}

	volatile float DegreeArray[3];
	volatile int16_t DeagreeArray_int[3];
	//NRF_LOG_INFO("------------------------ dof3_buf[%d][%d][%d]\n\r",dof3_buf[0]*1000,dof3_buf[1]*1000,dof3_buf[2]*1000);
	//NRF_LOG_INFO("------------------------ acc x[%d] y[%d] z[%d]",(int32_t)(dof3_buf[4]*1000),(int32_t)(dof3_buf[3]*1000),(int32_t)(dof3_buf[5]*1000));
	//NRF_LOG_INFO(" gyro x[%d] y[%d] z[%d]\n\r",(int32_t)(dof3_buf[1]*1000),(int32_t)(dof3_buf[0]*1000),(int32_t)(dof3_buf[2]*1000));
	MadgwickAHRSupdate(dof3_buf[4],dof3_buf[3],dof3_buf[5],dof3_buf[1],dof3_buf[0],dof3_buf[2],0.00001f,0.00001f,0.00001f);
	QuaternionToDegreeFast(DegreeArray);
	DeagreeArray_int[0] =  DegreeArray[0];
	DeagreeArray_int[1] =  DegreeArray[1];
	DeagreeArray_int[2] =  DegreeArray[2];
	NRF_LOG_INFO("---- Deagree [%5d][%5d][%5d]\n\r",DeagreeArray_int[0],DeagreeArray_int[1],DeagreeArray_int[2]);

#ifdef ONLY_TRANSFER_3DOF_DATA
	memset(sensor_data,0x00, sizeof(sensor_data));
	memcpy(sensor_data,key_sum,4);
	memcpy(sensor_data+4,touch_sum,4);
	memset(key_sum,0x00, sizeof(key_sum));
	memset(touch_sum,0x00, sizeof(touch_sum));

	sensor_data[8] = DATA_3DOF;//1----gyro data
	memcpy(sensor_data+9,(int8_t*)(DeagreeArray_int),2/*sizeof(float)*3+1*/);
	memcpy(sensor_data+11,(int8_t*)(DeagreeArray_int+1),2/*sizeof(float)*3+1*/);
	memcpy(sensor_data+13,(int8_t*)(DeagreeArray_int+2),2/*sizeof(float)*3+1*/);
#else
	memset(sensor_data,0x00, sizeof(sensor_data));
	static uint16_t time_stamp=0;
	void *p=sensor_data;
	time_stamp++;
	if(time_stamp==512)
		time_stamp=0;
//time_stamp 0-8
	//set_bits(0,8,time_stamp);
	set_bits(0,8,time_stamp);
	//(*((uint16_t*)p + TIME_STAMP_START)) |= time_stamp;
//packet id
	static uint16_t packet_id=0;
	packet_id++;
	if(packet_id == 32)
		packet_id=0;
	//(*(uint8_t)(p + PACKET_FLAG_START)) |= packet_id;
	//set_bits(9,13,packet_id);
	set_bits(9,13,packet_id);
//mag 13 13 13

	set_bits(14,26,DeagreeArray_int[0]);
	set_bits(27,39,DeagreeArray_int[1]);
	set_bits(40,52,DeagreeArray_int[2]);

//acc 13 13 13
	data1[0] >>=3;
	data1[1] >>=3;
	data1[2] >>=3;
	data1[3] >>=3;
	data1[4] >>=3;
	data1[5] >>=3;
	set_bits(53,65,data1[0]);
	set_bits(66,78,data1[1]);
	set_bits(79,91,data1[2]);

//gyro 13 13 13
	set_bits(92,104,data1[3]);
	set_bits(105,117,data1[4]);
	set_bits(118,130,data1[5]);

#endif
		}
	//if(sensor_data[0]!=NON_DATA || sensor_data[13]!=NON_DATA || sensor_data[26]!=NON_DATA){
		custom_on_send(m_conn_handle,&m_bas,sensor_data,sizeof(sensor_data));
	//}
	}

#endif

//#define OPEN_LOG_TIME
#ifdef OPEN_LOG_TIME
	if(systick_s_lastest.time >= systick_s.time){
		det_tick1 = systick_s_lastest.time - systick_s.time;
	}
	NRF_LOG_INFO("----- ---------------------- systick[%d] lastest[%d] det[%d] us[%d]\r\n",systick_s.time,systick_s_lastest.time,det_tick1,(det_tick1/64));
	systick_s_lastest.time = systick_s.time;
#endif
#if 0
	nrf_drv_systick_get(&systick_s_end);
	if(systick_s.time >= systick_s_end.time){
		det_tick2 = systick_s.time - systick_s_end.time;
	}
	NRF_LOG_INFO("----- ---------------------- start_systick[%d] end_systick[%d] det[%d] us[%d]\r\n",systick_s.time,systick_s_end.time,det_tick2,(det_tick2/64));
#endif
}
void SENSOR_INIT_1(void)
{
	app_timer_create(&sensor_poll_timer_id,APP_TIMER_MODE_SINGLE_SHOT,sensor_data_poll_handler);
}

int8_t I2C1_Read_Addr8(	const uint8_t slave_addr,const uint8_t read_addr,uint8_t *data,uint8_t data_num)
{
		ret_code_t err_code;
		
	  err_code = nrf_drv_twi_tx(&m_twi1, slave_addr, &read_addr, 1, false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
		err_code = nrf_drv_twi_rx(&m_twi1, slave_addr, data, data_num);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}
int8_t I2C1_Write_Addr8(	const uint8_t slave_addr,uint8_t write_addr,uint8_t * data,uint8_t data_num)
{
		ret_code_t err_code;
	  err_code = nrf_drv_twi_tx(&m_twi1, slave_addr, &write_addr, 1, false);
	  err_code = nrf_drv_twi_tx(&m_twi1, slave_addr, data, data_num, false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}
int8_t I2C2_Write_Addr8(	const uint8_t slave_addr,uint8_t write_addr,uint8_t write_value)
{
		ret_code_t err_code;
	  uint8_t write[2]={write_addr,write_value};	
	  err_code = nrf_drv_twi_tx(&m_twi1, slave_addr, write, sizeof(write), false);
	  if (err_code == NRF_SUCCESS){
				//nrf_drv_gpiote_out_set(PIN_OUT);
    }
	  return (int8_t)err_code;
}
void bmg160_init(void)
{
	uint8_t data=0;	
	I2C1_Read_Addr8(0x68,0x00,&data,1);
    NRF_LOG_INFO("-----------bmg160 read id 0x%x\r\n",data);
	data =3;
	I2C2_Write_Addr8(0x68,0x40,0x11);
	data = 0;
	I2C1_Read_Addr8(0x68,0x03,&data,1);
    NRF_LOG_INFO("-----------bmg160 read 0x6c [0x%x]\r\n",data);
}
void sensor_poll_start();
void sensor_cal_status()
{
	uint32_t read_gryo_offset[2]={0x00};
	fds_read(read_gryo_offset);
	if(!(read_gryo_offset[1] & 0xFFFF0000)){
		sw3153_light_select(RED, BLINK_LEVEL_0);
		nrf_delay_ms(500);
	}

}
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
	nrf_delay_ms(500);
    buttons_leds_init(&erase_bonds);
 #ifdef USE_SHADOW_CREATE
	nrf_gpio_cfg_output(TOUCH_RST_PIN);
	nrf_gpio_pin_write(TOUCH_RST_PIN,0);
	nrf_delay_ms(10);
	nrf_gpio_pin_write(TOUCH_RST_PIN,1);
	nrf_delay_ms(50);

	 //nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    nrf_drv_gpiote_in_init(TOUCH_INT_PIN, &in_config, in_pin_handler);
  	//nrf_drv_gpiote_in_init(SENSOR_INT_PIN, &in_config, sensor_in_pin_handler);
	nrf_drv_gpiote_in_event_enable(TOUCH_INT_PIN, true);
	//nrf_drv_gpiote_in_event_enable(SENSOR_INT_PIN, true);
	twi_init();
	twi_init_1();
	bmg160_init();
	uint16_t tp_version = tp_firmware_version();
    NRF_LOG_INFO("----- tp_version  [0x%x]\r\n",tp_version);
 #endif
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();
    // Start execution.
    NRF_LOG_INFO("HID Mouse example started. erase_bonds %d\r\n",erase_bonds);
    timers_start();
	saadc_init();
	
	nrf_drv_systick_init();
	sw3153_config();
	fds_test_init();
	sensor_cal_status();
	

	touch_timer_init();
	mouse_slow_init();
	connect_sleep_init();
	saadc_sample_init();
	touch_atcion_detect_init();

	SENSOR_INIT();
	SENSOR_INIT_1();
	sensor_poll_start();
	saadc_sample_start();
	if(mode_will_cal == true){
		//Mode_switch(MODE_CALIBRATE,false);
		sensor_cal_init();
		sensor_cal_start();
	}
	bmi160_cal_offset_apply();

		//erase_bonds = true;
    advertising_start(erase_bonds);

	sw3153_light_select(BLUE, BLINK_LEVEL_2);
	if(mode_will_test == true){
		Mode_switch(MODE_TEST,false);
		//Mode_switch(false,false,true);
	}
    // Enter main loop.
	for (;;)
    {
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
