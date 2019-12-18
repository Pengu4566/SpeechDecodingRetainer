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
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "ble_advertising.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// From HID Keyboard
#include "nrf_assert.h"

// From CSENSE
#include "boards.h"
#include "nrf_csense.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"



/* CSENSE Definitions */

/* Time between RTC interrupts. */
#define APP_TIMER_TICKS_TIMEOUT APP_TIMER_TICKS(50)       // *** Was 50, changed to 30, went back to 50

/* Scale range. */
#define RANGE                   50

/* Analog inputs. */
#define AIN0                    0
#define AIN1                    1
#define AIN2                    2
#define AIN3                    3
#define AIN4                    4
#define AIN5                    5
#define AIN6                    6
#define AIN7                    7

/* Definition which pads use which analog inputs. */
#define BUTTON1                 AIN0    // *** Send/Return | 'abcd'
#define BUTTON2                 AIN1    // L-Click | 'efgh'
#define BUTTON3                 AIN2    // Left | 'ijkl'
#define BUTTON4                 AIN3    // *** Up | 'mnop'
#define BUTTON5                 AIN4    // Down | 'qrst'
#define BUTTON6                 AIN5    // Right |'uvwx'
#define BUTTON7                 AIN6    // R-Click | 'yz'
#define BUTTON8                 AIN7    // Delete | Caps (double-click) | Change (long)

/* Threshold values for pads and button. */
#define THRESHOLD_PAD_1         40 // 30 // 20 // 50 // 30 // 300 // 100 //15 //5 //2 //4 //50 //// 80 // 10 //80    // L - P0.03 - AIN1           // Worked when all at 4 at 
// 2000 / 4400 / 1850*
#define THRESHOLD_PAD_2         40 // 30 // 20 // 50 // 30 // 300 // 100 //15 //5 //2 //4 //50 // 100 // 50    // U - P0.04 - AIN2           // Worked when all at 4 at 1250 (threshold is 1250 for this one?)
// 1800 / 4200 / 1250*
#define THRESHOLD_PAD_3         40 // 30 // 20 // 30 // 45 // 50 // 30 // 300 // 100 //15 //5 //2 //4 //50 // 1000 // 50     // D - P0.29 - AIN5           // Worked when all at 4 at 1250 (threshold is 1250 for this one?)
// 2000 / 4400 / 850*
#define THRESHOLD_PAD_4         40 // 30 // 20 // 45 // 50 // 30 // 300 // 100 //15 //5 //2 //4 //50 ////  70 // 10000 // 70    // R - P0.30 - AIN6           // Worked when all at 4 at 1250 (threshold is 1250 for this one?)
// 1800 / 1300 / 1150*

#define THRESHOLD_PAD_5         40 // 30 // 20 // 5 // 50 // 100 // 30 // 50 // 30 // 300 // 100 //15 //5 //2 //4 //50 //// 60 // 5000 // 60    // L.C. - P0.31 - AIN7
//500 / 1050*     // L.C. - P0.02 - AIN0
#define THRESHOLD_PAD_6         40 // 30 // 20 // 5 // 50 // 100 // 30 // 50 // 30 // 300 // 100 //15 //5 //2 //4 //50 //// 60 // 2500 // 60    // R.C. - P0.28 - AIN4
// 1050*
#define THRESHOLD_PAD_7         40 // 20 // 50 // 30 // 600 // 1000 //15 //5 //2 //4 //50 //// 100 // 950 // 7500 // 950    //  //**********   this pad is a bit wonky on the nrf52 dk (2017 one)
// 1050*
#define THRESHOLD_PAD_8         40 // 35 // 20 // 50 // 30 // 300 //100 //15 //5 //2 //4 //50 //// 65 // 100,000 // 65    // 
// 1050*

/*lint -e19 -save */
//NRF_CSENSE_BUTTON_DEF(m_button1, (BUTTON1, THRESHOLD_PAD_1));   // L - P0.03 - AIN1
NRF_CSENSE_BUTTON_DEF(m_button2, (BUTTON2, THRESHOLD_PAD_2));   // U - P0.04 - AIN2
NRF_CSENSE_BUTTON_DEF(m_button3, (BUTTON3, THRESHOLD_PAD_3));   // D - P0.29 - AIN5
NRF_CSENSE_BUTTON_DEF(m_button4, (BUTTON4, THRESHOLD_PAD_4));   // R - P0.30 - AIN6
NRF_CSENSE_BUTTON_DEF(m_button5, (BUTTON5, THRESHOLD_PAD_5));   // L.C. - P0.31 - AIN7          // L.C. - P0.02 - AIN0
NRF_CSENSE_BUTTON_DEF(m_button6, (BUTTON6, THRESHOLD_PAD_6));   // R.C. - P0.28 - AIN4
NRF_CSENSE_BUTTON_DEF(m_button7, (BUTTON7, THRESHOLD_PAD_7));
//NRF_CSENSE_BUTTON_DEF(m_button8, (BUTTON8, THRESHOLD_PAD_8));   // Change
/*lint -restore*/





/* Functions from HID Keyboard

BUFFER_LIST_INIT()
BUFFER_LIST_FULL()
BUFFER_LIST_EMPTY()
BUFFER_ELEMENT_INIT(i)

typedef struct hid_key_buffer
STATIC_ASSERT(sizeof(buffer_entry_t) % 4 == 0);
typedef struct    // circular buffer list

static uint8_t m_sample_key_press_scan_str[] = //< Key pattern to be sent when the key press button has been pushed. 
static uint8_t m_caps_on_key_scan_str[] = //< Key pattern to be sent when the output report has been written with the CAPS LOCK bit set. 
static uint8_t m_caps_off_key_scan_str[] = //< Key pattern to be sent when the output report has been written with the CAPS LOCK bit cleared.

static void gap_params_init(void)
// Has one line different (calls BLE_APPEARANCE_HID_KEYBOARD instead of BLE_APPEARANCE_HID_MOUSE)

static void hids_init(void)
// Function for initializing HID service




static uint32_t send_key_scan_press_release(....)
// Function for transmitting a key scan press and release notification

static void buffer_init(void)
// Function for initializing buffer queue used to key events that could not be transmitted

static uint32_t buffer_enqueue(....)
// Function for enqueing key scan patterns that could not be transmitted completely or partially

static uint32_t buffer_dequeue(bool tx_flag)
// Function to dequeue key scan patterns that could not be transmitted completely or partially

static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)   (?? required)
// Function for sending sample key presses to the peer

static void on_hid_rep_char_write(ble_hids_evt_t * p_evt)
// Function for handling the HID report characteristic write event        (?? required)




static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
// Already added the one different line from HID Keyboard
          case BLE_HIDS_EVT_REP_CHAR_WRITE:
            on_hid_rep_char_write(p_evt);
            break;


static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
// Function for handling BLE Events
*** Has one or two additional cases related to buffer / send key press(?)



int main(void)
// Main loop function
*** Has one additional line for buffer initialization  buffer_init() 


*/



/* Definitions */

#define DEVICE_NAME                     "Nordic_MK"                                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
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
#define SLAVE_LATENCY                   15                                          /**< Slave latency. */
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

// HID Reports
#define MOVEMENT_SPEED                  20                                          /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              4                                           /**< Number of input reports in this application. */

#define INPUT_REPORT_KEYS_MAX_LEN       8                                           /**< Maximum length of the Input (Keyboard) Report characteristic. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define OUTPUT_REPORT_MAX_LEN           1                                           /**< Maximum length of Output (Keyboard) Report. */

#define INPUT_REPORT_KEYS_INDEX         0                                           /**< Index of Input (Keyboard) Report. */
#define INPUT_REP_BUTTONS_INDEX         1                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        2                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         3                                           /**< Index of Mouse Input Report containing media player data. */
#define OUTPUT_REPORT_INDEX             0                                           /**< Index of Output (Keyboard) Report. */

#define INPUT_REP_REF_ID                1                                           /**< Id of reference to Keyboard Input Report. */
#define INPUT_REP_REF_BUTTONS_ID        2                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       3                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        4                                           /**< Id of reference to Mouse Input Report containing media player data. */
#define OUTPUT_REP_REF_ID               0                                           /**< Id of reference to Keyboard Output Report. */


#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVENT_DATA_SIZE             /**< Maximum size of scheduler events. */
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
#define APP_ADV_SLOW_TIMEOUT            180 //180 //*****                                         /**< The duration of the slow advertising period (in seconds). */


APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */
BLE_HIDS_DEF(m_hids_mk);                                                            /**< HID service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */


static bool              m_in_boot_mode = false;                                    /**< Current protocol mode. */
static uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                                 /**< Device reference handle to the current bonded central. */
static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */
static pm_peer_id_t      m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];       /**< List of peers currently in the whitelist. */
static uint32_t          m_whitelist_peer_cnt;                                      /**< Number of peers currently in the whitelist. */
static ble_uuid_t        m_adv_uuids[] =                                            /**< Universally unique service identifiers. */
{
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
};




/* HID Keyboard */

#define SHIFT_BUTTON_ID                     1                                          /**< Button used as 'SHIFT' Key. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK    0x02                                       /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */  //*****
#define MAX_BUFFER_ENTRIES                  5                                          /**< Number of elements that can be enqueued */

#define MODIFIER_KEY_POS                    0                                          /**< Position of the modifier byte in the Input Report. */
#define SCAN_CODE_POS                       2                                          /**< The start position of the key scan code in a HID Report. */
#define SHIFT_KEY_CODE                      0x02                                       /**< Key code indicating the press of the Shift Key. */

#define MAX_KEYS_IN_ONE_REPORT              (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS)/**< Maximum number of key presses that can be sent in one Input Report. */

static bool              m_caps_on = false;                         /**< Variable to indicate if Caps Lock is turned on. */

//static buffer_list_t     buffer_list;                               /**< List to enqueue not just data to be sent, but also related information like the handle, connection handle etc */



// Additions
 
static bool              m_change = false;                             /**< Variable to indicate if Change Key is turned on.   (TRUE = Keyboard Input) */

APP_TIMER_DEF(m_keyboard_input_id);                                   /**< Timer for T9 input memory */
#define KEYBOARD_INPUT_INCREMENT        APP_TIMER_TICKS(1500)         /**< Keyboard increment interval (ticks) */

APP_TIMER_DEF(m_double_click_id);                                     /**< Timer for double clicks */
#define DOUBLE_CLICK                    APP_TIMER_TICKS(400)          /**< Double click interval (ticks) */

APP_TIMER_DEF(m_long_hold_id);                                        /**< Timer for long holds */
#define LONG_HOLD                       APP_TIMER_TICKS(2000)         /**< Long hold interval (ticks) */

static bool              m_keyboard_increment = false;                /**< Bool for incrementing keyboard key input (T9)  */
static bool              m_double_click = false;
static bool              m_long_hold = false;

static uint8_t           m_last_keyboard_input;                       /**< Stores last keyboard input key ('a', 'e', 'i', 'm', 'q', 'u', 'y') */
static uint8_t           m_last_double_click;                         /**< Stores last double click input button (1 through 8) */


static uint8_t m_abcd_key_press_scan_str[] = /**< Key pattern to be sent when ... */
{
    0x04,       /* Key a */
    0x2a,
    0x05,       /* Key b */
    0x2a,
    0x06,       /* Key c */
    0x2a,
    0x07,       /* Key d */
    0x2a,
};

static uint8_t m_efgh_key_press_scan_str[] = /**< Key pattern to be sent when ... */
{
    0x08,       /* Key e */
    0x2a,
    0x09,       /* Key f */
    0x2a,
    0x0a,       /* Key g */
    0x2a,
    0x0b,       /* Key h */
    0x2a,
};

static uint8_t m_ijkl_key_press_scan_str[] = /**< Key pattern to be sent when ... */
{
    0x0c,       /* Key i */
    0x2a,
    0x0d,       /* Key j */
    0x2a,
    0x0e,       /* Key k */
    0x2a,
    0x0f,       /* Key l */
    0x2a,
};

static uint8_t m_mnop_key_press_scan_str[] = /**< Key pattern to be sent when ... */
{
    0x10,       /* Key m */
    0x2a,
    0x11,       /* Key n */
    0x2a,
    0x12,       /* Key o */
    0x2a,
    0x13,       /* Key p */
    0x2a,
};

static uint8_t m_qrst_key_press_scan_str[] = /**< Key pattern to be sent when ... */
{
    0x14,       /* Key q */
    0x2a,
    0x15,       /* Key r */
    0x2a,
    0x16,       /* Key s */
    0x2a,
    0x17,       /* Key t */
    0x2a,
};

static uint8_t m_uvwx_key_press_scan_str[] = /**< Key pattern to be sent when ... */
{
    0x18,       /* Key u */
    0x2a,
    0x19,       /* Key v */
    0x2a,
    0x1a,       /* Key w */
    0x2a,
    0x1b,       /* Key x */
    0x2a,
};

static uint8_t m_yz_key_press_scan_str[] = /**< Key pattern to be sent when ... */
{
    0x1c,       /* Key y */
    0x2a,
    0x1d,       /* Key z */
    0x2a,
};

static uint8_t m_del_key_press_scan_str[] = {0x2a};   /**< Delete key **/

static uint8_t m_ent_key_press_scan_str[] = {0x28};   /**< Enter key **/

/* HID Keyboard End */





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

    NRF_LOG_INFO("Erase bonds!");

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

        ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
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
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);

            m_peer_id = p_evt->peer_id;
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
            // ***** pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
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

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                               m_whitelist_peer_cnt + 1,
                               BLE_GAP_WHITELIST_ADDR_MAX_COUNT);
                // Note: You should check on what kind of white list policy your application should use.

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                    // The whitelist has been modified, update it in the Peer Manager.
                    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);

                    err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    if (err_code != NRF_ERROR_NOT_SUPPORTED)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                }
            }
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
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}





/**@brief   Function for transmitting a key scan Press & Release Notification.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_instance     Identifies the service for which Key Notifications are requested.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern. 0 < pattern_len < 7.
 * @param[in]  pattern_offset Offset applied to Key Pattern for transmission.
 * @param[out] actual_len     Provides actual length of Key Pattern transmitted, making buffering of
 *                            rest possible if needed.
 * @return     NRF_SUCCESS on success, NRF_ERROR_RESOURCES in case transmission could not be
 *             completed due to lack of transmission buffer or other error codes indicating reason
 *             for failure.
 *
 * @note       In case of NRF_ERROR_RESOURCES, remaining pattern that could not be transmitted
 *             can be enqueued \ref buffer_enqueue function.
 *             In case a pattern of 'cofFEe' is the p_key_pattern, with pattern_len as 6 and
 *             pattern_offset as 0, the notifications as observed on the peer side would be
 *             1>    'c', 'o', 'f', 'F', 'E', 'e'
 *             2>    -  , 'o', 'f', 'F', 'E', 'e'
 *             3>    -  ,   -, 'f', 'F', 'E', 'e'
 *             4>    -  ,   -,   -, 'F', 'E', 'e'
 *             5>    -  ,   -,   -,   -, 'E', 'e'
 *             6>    -  ,   -,   -,   -,   -, 'e'
 *             7>    -  ,   -,   -,   -,   -,  -
 *             Here, '-' refers to release, 'c' refers to the key character being transmitted.
 *             Therefore 7 notifications will be sent.
 *             In case an offset of 4 was provided, the pattern notifications sent will be from 5-7
 *             will be transmitted.
 */
static uint32_t send_key_scan_press_release(ble_hids_t * p_hids,              // Calls ble_hids_inp_rep_send()
                                            uint8_t    * p_key_pattern,
                                            uint16_t     pattern_len,
                                            uint16_t     pattern_offset,
                                            uint16_t   * p_actual_len)
{
    ret_code_t err_code;
    uint16_t offset;
    uint16_t data_len;
    uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];

    // HID Report Descriptor enumerates an array of size 6, the pattern hence shall not be any
    // longer than this.
    STATIC_ASSERT((INPUT_REPORT_KEYS_MAX_LEN - 2) == 6);

    ASSERT(pattern_len <= (INPUT_REPORT_KEYS_MAX_LEN - 2));

    offset   = pattern_offset;
    data_len = pattern_len;

    do
    {
        // Reset the data buffer.
        memset(data, 0, sizeof(data));

        // Copy the scan code.
        memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);

//        if (bsp_button_is_pressed(SHIFT_BUTTON_ID))
//        {
//            data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
//        }

        // Added caps handling for csense events
        if (m_caps_on == true)
        {
            data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
        }

        if (!m_in_boot_mode)
        {
            err_code = ble_hids_inp_rep_send(p_hids,
                                             INPUT_REPORT_KEYS_INDEX,
                                             INPUT_REPORT_KEYS_MAX_LEN,
                                             data);
        }
        else
        {
            err_code = ble_hids_boot_kb_inp_rep_send(p_hids,
                                                     INPUT_REPORT_KEYS_MAX_LEN,
                                                     data);
        }

        if (err_code != NRF_SUCCESS)
        {
            break;
        }

        offset++;
    }
    while (offset <= data_len);

    *p_actual_len = offset;

    return err_code;
}






/**@brief Function for sending sample key presses to the peer.    // Calls  send_key_scan_press_release()
 *
 * @param[in]   key_pattern_len   Pattern length.
 * @param[in]   p_key_pattern     Pattern to be sent.
 */
static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)
{
    ret_code_t err_code;
    uint16_t actual_len;

    err_code = send_key_scan_press_release(&m_hids_mk,
                                           p_key_pattern,
                                           key_pattern_len,
                                           0,
                                           &actual_len);
    // An additional notification is needed for release of all keys, therefore check
    // is for actual_len <= key_pattern_len and not actual_len < key_pattern_len.
    if ((err_code == NRF_ERROR_RESOURCES) && (actual_len <= key_pattern_len))
    {
        // Buffer enqueue routine return value is not intentionally checked.
        // Rationale: Its better to have a a few keys missing than have a system
        // reset. Recommendation is to work out most optimal value for
        // MAX_BUFFER_ENTRIES to minimize chances of buffer queue full condition
        
        // UNUSED_VARIABLE(buffer_enqueue(&m_hids, p_key_pattern, key_pattern_len, actual_len));
    }


    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
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


/* Function for handling keyboard increment timer */
static void keyboard_input_timeout_handler(void * p_context)
{
    //UNUSED_PARAMETER(p_context);
    m_keyboard_increment = false;
}


/* Function for handling double click timer */
static void double_click_timeout_handler(void * p_context)
{
    //UNUSED_PARAMETER(p_context);
    uint8_t * p_key = m_del_key_press_scan_str;
    keys_send(1,p_key);

    m_double_click = false;
}


/* Function for handling long hold timer */
static void long_hold_timeout_handler(void * p_context)
{
    //UNUSED_PARAMETER(p_context);
    m_long_hold = true;
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

    // Create keyboard increment input timer.
    err_code = app_timer_create(&m_keyboard_input_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                keyboard_input_timeout_handler);

    // Create double click input timer.
    err_code = app_timer_create(&m_double_click_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                double_click_timeout_handler);

    // Create long hold input timer.
    err_code = app_timer_create(&m_long_hold_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                long_hold_timeout_handler);

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

    //err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
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
}






/**@brief Function for initializing HID Service. (combined)
 */
static void hids_init_mk(void)
{
    ret_code_t                 err_code;
    ble_hids_init_t            hids_init_obj;
    ble_hids_inp_rep_init_t    input_report_array[INPUT_REPORT_COUNT];        // ***** Why is size 1 ??? - changed to INPUT_REPORT_COUNT (4)
    ble_hids_inp_rep_init_t  * p_input_report;
    ble_hids_outp_rep_init_t   output_report_array[1];
    ble_hids_outp_rep_init_t * p_output_report;
    uint8_t                    hid_info_flags;

    //memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));
    memset((void *)input_report_array, 0, sizeof(input_report_array));
    memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));


    static uint8_t report_map_data[] =
    {
        // Report ID 1: Keyboard

        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x06,       // Usage (Keyboard)
        0xA1, 0x01,       // Collection (Application)

        0x85, INPUT_REP_REF_ID,   // Report ID (1)
        0x05, 0x07,               // Usage Page (Key Codes)                   // Usage is 'key codes'
        0x19, 0xe0,               // Usage Minimum (224)                      // ?? Describes key codes, min 224 (??)
        0x29, 0xe7,               // Usage Maximum (231)                      // ?? Describes key codes, max 231 (??)
        0x15, 0x00,               // Logical Minimum (0)                      // Each button status is 0 or 1
        0x25, 0x01,               // Logical Maximum (1)                      // Each button status is 0 or 1

        0x75, 0x01,               // Report Size (1)                          // Number of Bytes (?)
        0x95, 0x08,               // Report Count (8)                         // Number of Bits used
        0x81, 0x02,               // Input (Data, Variable, Absolute)         // Send variable data to the computer

        0x95, 0x01,               // Report Count (1)
        0x75, 0x08,               // Report Size (8)
        0x81, 0x01,               // Input (Constant) reserved byte(1)

        0x95, 0x05,               // Report Count (5)
        0x75, 0x01,               // Report Size (1)

        0x05, 0x08,               // Usage Page (Page# for LEDs)
        0x19, 0x01,               // Usage Minimum (1)
        0x29, 0x05,               // Usage Maximum (5)
        0x91, 0x02,               // Output (Data, Variable, Absolute), Led report

        0x95, 0x01,               // Report Count (1)
        0x75, 0x03,               // Report Size (3)
        0x91, 0x01,               // Output (Data, Variable, Absolute), Led report padding

        0x95, 0x06,               // Report Count (6)
        0x75, 0x08,               // Report Size (8)
        0x15, 0x00,               // Logical Minimum (0)
        0x25, 0x65,               // Logical Maximum (101)
        0x05, 0x07,               // Usage Page (Key codes)
        0x19, 0x00,               // Usage Minimum (0)
        0x29, 0x65,               // Usage Maximum (101)
        0x81, 0x00,               // Input (Data, Array) Key array(6 bytes)

        0x09, 0x05,               // Usage (Vendor Defined)
        0x15, 0x00,               // Logical Minimum (0)
        0x26, 0xFF, 0x00,         // Logical Maximum (255)
        0x95, 0x02,               // Report Count (2)
        0x75, 0x08,               // Report Size (8 bit)
        0xB1, 0x02,               // Feature (Data, Variable, Absolute)

        0xC0,             // ** End Collection (Application)


        // Report ID 2: Mouse buttons + scroll/pan
        // Report ID 3: Mouse motion

        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x02,       // Usage (Mouse)
        0xA1, 0x01,       // Collection (Application)

        0x85, INPUT_REP_REF_BUTTONS_ID, //     Report Id (2)
        0x09, 0x01,               //     Usage (Pointer)
        0xA1, 0x00,               //     Collection (Physical)
        0x95, 0x05,               //         Report Count (3)  (5?)
        0x75, 0x01,               //         Report Size (1)
        0x05, 0x09,               //         Usage Page (Buttons)
        0x19, 0x01,               //             Usage Minimum (01)
        0x29, 0x05,               //             Usage Maximum (05)
        0x15, 0x00,               //             Logical Minimum (0)
        0x25, 0x01,               //             Logical Maximum (1)
        0x81, 0x02,               //             Input (Data, Variable, Absolute)
        0x95, 0x01,               //             Report Count (1)
        0x75, 0x03,               //             Report Size (3)
        0x81, 0x01,               //             Input (Constant) for padding
        0x75, 0x08,               //             Report Size (8)
        0x95, 0x01,               //             Report Count (1)
        0x05, 0x01,               //         Usage Page (Generic Desktop)
        0x09, 0x38,               //             Usage (Wheel)
        0x15, 0x81,               //             Logical Minimum (-127)
        0x25, 0x7F,               //             Logical Maximum (127)
        0x81, 0x06,               //             Input (Data, Variable, Relative)
        0x05, 0x0C,               //         Usage Page (Consumer)
        0x0A, 0x38, 0x02,         //             Usage (AC Pan)
        0x95, 0x01,               //             Report Count (1)
        0x81, 0x06,               //             Input (Data,Value,Relative,Bit Field)
        0xC0,                     //     End Collection (Physical)

        // Report ID 3: Mouse motion
        0x85, INPUT_REP_REF_MOVEMENT_ID, //     Report Id (3)
        0x09, 0x01,               //     Usage (Pointer)
        0xA1, 0x00,               //     Collection (Physical)
        0x75, 0x0C,               //         Report Size (12)
        0x95, 0x02,               //         Report Count (2)
        0x05, 0x01,               //         Usage Page (Generic Desktop)
        0x09, 0x30,               //             Usage (X)
        0x09, 0x31,               //             Usage (Y)
        0x16, 0x01, 0xF8,         //             Logical maximum (2047)
        0x26, 0xFF, 0x07,         //             Logical minimum (-2047)
        0x81, 0x06,               //             Input (Data, Variable, Relative)
        0xC0,                     //     End Collection (Physical)
        0xC0,             // End Collection (Application)

        // Report ID 4: Advanced buttons
        0x05, 0x0C,       // Usage Page (Consumer)
        0x09, 0x01,       // Usage (Consumer Control)
        0xA1, 0x01,       // Collection (Application)
        0x85, INPUT_REP_REF_MPLAYER_ID,       // Report Id (4)
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
        0xC0              // End Collection

    };

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);



    p_input_report                      = &input_report_array[INPUT_REP_BUTTONS_INDEX];
    p_input_report->max_len             = INPUT_REP_BUTTONS_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_BUTTONS_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &input_report_array[INPUT_REP_MOVEMENT_INDEX];
    p_input_report->max_len             = INPUT_REP_MOVEMENT_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOVEMENT_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &input_report_array[INPUT_REP_MPLAYER_INDEX];
    p_input_report->max_len             = INPUT_REP_MEDIA_PLAYER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);


    p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);



    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    //hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = true;                        // ***** Can both be set true????
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;          // ***** Was 1 for keyboard - set to INPUT_REPORT_COUNT (4)
    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.outp_rep_count                 = 1;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids_mk, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}







/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    dis_init();
    bas_init();
    hids_init_mk();
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







///**@brief   Function for transmitting a key scan Press & Release Notification.
// *
// * @warning This handler is an example only. You need to analyze how you wish to send the key
// *          release.
// *
// * @param[in]  p_instance     Identifies the service for which Key Notifications are requested.
// * @param[in]  p_key_pattern  Pointer to key pattern.
// * @param[in]  pattern_len    Length of key pattern. 0 < pattern_len < 7.
// * @param[in]  pattern_offset Offset applied to Key Pattern for transmission.
// * @param[out] actual_len     Provides actual length of Key Pattern transmitted, making buffering of
// *                            rest possible if needed.
// * @return     NRF_SUCCESS on success, NRF_ERROR_RESOURCES in case transmission could not be
// *             completed due to lack of transmission buffer or other error codes indicating reason
// *             for failure.
// *
// * @note       In case of NRF_ERROR_RESOURCES, remaining pattern that could not be transmitted
// *             can be enqueued \ref buffer_enqueue function.
// *             In case a pattern of 'cofFEe' is the p_key_pattern, with pattern_len as 6 and
// *             pattern_offset as 0, the notifications as observed on the peer side would be
// *             1>    'c', 'o', 'f', 'F', 'E', 'e'
// *             2>    -  , 'o', 'f', 'F', 'E', 'e'
// *             3>    -  ,   -, 'f', 'F', 'E', 'e'
// *             4>    -  ,   -,   -, 'F', 'E', 'e'
// *             5>    -  ,   -,   -,   -, 'E', 'e'
// *             6>    -  ,   -,   -,   -,   -, 'e'
// *             7>    -  ,   -,   -,   -,   -,  -
// *             Here, '-' refers to release, 'c' refers to the key character being transmitted.
// *             Therefore 7 notifications will be sent.
// *             In case an offset of 4 was provided, the pattern notifications sent will be from 5-7
// *             will be transmitted.
// */
//static uint32_t send_key_scan_press_release(ble_hids_t * p_hids,
//                                            uint8_t    * p_key_pattern,
//                                            uint16_t     pattern_len,
//                                            uint16_t     pattern_offset,
//                                            uint16_t   * p_actual_len)
//{
//    ret_code_t err_code;
//    uint16_t offset;
//    uint16_t data_len;
//    uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];
//
//    // HID Report Descriptor enumerates an array of size 6, the pattern hence shall not be any
//    // longer than this.
//    STATIC_ASSERT((INPUT_REPORT_KEYS_MAX_LEN - 2) == 6);
//
//    ASSERT(pattern_len <= (INPUT_REPORT_KEYS_MAX_LEN - 2));
//
//    offset   = pattern_offset;
//    data_len = pattern_len;
//
//    do
//    {
//        // Reset the data buffer.
//        memset(data, 0, sizeof(data));
//
//        // Copy the scan code.
//        memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);
//
//        if (bsp_button_is_pressed(SHIFT_BUTTON_ID))
//        {
//            data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
//        }
//
//        // Added caps handling for csense events
//        if (m_caps_on == true)
//        {
//            data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
//        }
//
//        if (!m_in_boot_mode)
//        {
//            err_code = ble_hids_inp_rep_send(p_hids,
//                                             INPUT_REPORT_KEYS_INDEX,
//                                             INPUT_REPORT_KEYS_MAX_LEN,
//                                             data);
//        }
//        else
//        {
//            err_code = ble_hids_boot_kb_inp_rep_send(p_hids,
//                                                     INPUT_REPORT_KEYS_MAX_LEN,
//                                                     data);
//        }
//
//        if (err_code != NRF_SUCCESS)
//        {
//            break;
//        }
//
//        offset++;
//    }
//    while (offset <= data_len);
//
//    *p_actual_len = offset;
//
//    return err_code;
//}
//
//
//
//
//
//
///**@brief Function for sending sample key presses to the peer.
// *
// * @param[in]   key_pattern_len   Pattern length.
// * @param[in]   p_key_pattern     Pattern to be sent.
// */
//static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)
//{
//    ret_code_t err_code;
//    uint16_t actual_len;
//
//    err_code = send_key_scan_press_release(&m_hids_kb,
//                                           p_key_pattern,
//                                           key_pattern_len,
//                                           0,
//                                           &actual_len);
//    // An additional notification is needed for release of all keys, therefore check
//    // is for actual_len <= key_pattern_len and not actual_len < key_pattern_len.
//    if ((err_code == NRF_ERROR_RESOURCES) && (actual_len <= key_pattern_len))
//    {
//        // Buffer enqueue routine return value is not intentionally checked.
//        // Rationale: Its better to have a a few keys missing than have a system
//        // reset. Recommendation is to work out most optimal value for
//        // MAX_BUFFER_ENTRIES to minimize chances of buffer queue full condition
//        
//        // UNUSED_VARIABLE(buffer_enqueue(&m_hids, p_key_pattern, key_pattern_len, actual_len));
//    }
//
//
//    if ((err_code != NRF_SUCCESS) &&
//        (err_code != NRF_ERROR_INVALID_STATE) &&
//        (err_code != NRF_ERROR_RESOURCES) &&
//        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//       )
//    {
//        APP_ERROR_HANDLER(err_code);
//    }
//}




/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(ble_hids_evt_t * p_evt)
{
    if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT)
    {
        ret_code_t err_code;
        uint8_t  report_val;
        uint8_t  report_index = p_evt->params.char_write.char_id.rep_index;

        if (report_index == OUTPUT_REPORT_INDEX)      // = 0
        {
            // This code assumes that the outptu report is one byte long. Hence the following
            // static assert is made.
            STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);

            err_code = ble_hids_outp_rep_get(&m_hids_mk,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             &report_val);
            APP_ERROR_CHECK(err_code);

            if (!m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) != 0))
            {
                // Caps Lock is turned On.
                NRF_LOG_INFO("Caps Lock is turned On!");
                err_code = bsp_indication_set(BSP_INDICATE_ALERT_3);
                APP_ERROR_CHECK(err_code);

                // keys_send(sizeof(m_caps_on_key_scan_str), m_caps_on_key_scan_str);
                // m_caps_on = true;
            }
            else if (m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) == 0))
            {
                // Caps Lock is turned Off .
                NRF_LOG_INFO("Caps Lock is turned Off!");
                err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
                APP_ERROR_CHECK(err_code);

                // keys_send(sizeof(m_caps_off_key_scan_str), m_caps_off_key_scan_str);
                // m_caps_on = false;
            }
            else
            {
                // The report received is not supported by this application. Do nothing.
            }
        }
    }
}








/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
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

        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            on_hid_rep_char_write(p_evt);
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
            NRF_LOG_INFO("Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            sleep_mode_enter();
            ///// No change made, already sleeps after advertising

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
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                           addr_cnt,
                           irk_cnt);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
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
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.

            // Dequeue all keys without transmission.
            //(void) buffer_dequeue(false);

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Reset m_caps_on variable. Upon reconnect, the HID host will re-send the Output
            // report containing the Caps lock state.
            m_caps_on = false;
            // disabling alert 3. signal - used for capslock ON
            err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
            APP_ERROR_CHECK(err_code);

            /////** Start advertising after disconnect (but don't erase bonds); at end of advertising after no connection, will be put to sleep mode
            advertising_start(true);

            /////** Added command to send device to deep sleep mode after disconnect
            //sleep_mode_enter();

            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif


        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            // Send next key event
            // (void) buffer_dequeue(true);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);


            /////** Added command to send device to deep sleep mode after disconnect
            //sleep_mode_enter();

            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

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


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
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
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled      = true;
    init.config.ble_adv_directed_enabled       = true;
    init.config.ble_adv_directed_slow_enabled  = false;
    init.config.ble_adv_directed_slow_interval = 0;
    init.config.ble_adv_directed_slow_timeout  = 0;
    init.config.ble_adv_fast_enabled           = true;
    init.config.ble_adv_fast_interval          = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout           = APP_ADV_FAST_TIMEOUT;
    init.config.ble_adv_slow_enabled           = true;
    init.config.ble_adv_slow_interval          = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout           = APP_ADV_SLOW_TIMEOUT;

    //*****
    init.config.ble_adv_on_disconnect_disabled = true;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
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

        err_code = ble_hids_boot_mouse_inp_rep_send(&m_hids_mk,
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

        err_code = ble_hids_inp_rep_send(&m_hids_mk,
                                         INPUT_REP_MOVEMENT_INDEX,
                                         INPUT_REP_MOVEMENT_LEN,
                                         buffer);
    }

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}





/**@brief Function for sending a Mouse Click.
 *
 * @param[in]   
 * @param[in]   
 */
static void mouse_click_send(uint8_t x, uint8_t y, uint8_t z) 
{
    ret_code_t err_code;
      // 'ret_code_t' or 'uint32_t err_code;' (??)

    
    //if (m_in_boot_mode)    return;      // *****
    
    
    uint8_t buffer[INPUT_REP_BUTTONS_LEN];

    //APP_ERROR_CHECK_BOOL(INPUT_REP_MOVEMENT_LEN == 3);

    buffer[0] = x;  // Left button (bit 0) pressed
    buffer[1] = y;  // Scroll value (-127, 128)
    buffer[2] = z;  // Sideways scroll value (-127, 128)

    err_code = ble_hids_inp_rep_send(&m_hids_mk,
                                     INPUT_REP_BUTTONS_INDEX, 
                                     INPUT_REP_BUTTONS_LEN, 
                                     buffer);                                  

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&            // Or BLE_ERROR_NO_TX_BUFFERS  (??)
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);   // Or APP_ERROR_CHECK or APP_ERROR_HANDLER  (??)
    }
}





/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
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
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                mouse_movement_send(-MOVEMENT_SPEED, 0);
            }
            break;

        case BSP_EVENT_KEY_1:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                mouse_movement_send(0, -MOVEMENT_SPEED);
            }
            break;

        case BSP_EVENT_KEY_2:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                //mouse_movement_send(MOVEMENT_SPEED, 0);
                mouse_click_send(1,0,0);
                mouse_click_send(0,0,0);
            }
            break;

        case BSP_EVENT_KEY_3:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                //mouse_movement_send(0, MOVEMENT_SPEED);
                //mouse_click_send(2,0,0);
                //mouse_click_send(0,0,0);
                  uint8_t * p_key = m_abcd_key_press_scan_str;
                  keys_send(1,p_key);

            }
            break;

        default:
            break;
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

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}




/* CSENSE */


/**
 * @brief Event handler for Capacitive Sensor High module.
 *
 * @param [in] p_evt_type                    Pointer to event data structure.
 */
void nrf_csense_handler(nrf_csense_evt_t * p_evt)
{

    static uint8_t * p_key_a = m_abcd_key_press_scan_str;
    static uint8_t size_a = 0;

    static uint8_t * p_key_e = m_efgh_key_press_scan_str;
    static uint8_t size_e = 0;

    static uint8_t * p_key_i = m_ijkl_key_press_scan_str;
    static uint8_t size_i = 0;

    static uint8_t * p_key_m = m_mnop_key_press_scan_str;
    static uint8_t size_m = 0;

    static uint8_t * p_key_q = m_qrst_key_press_scan_str;
    static uint8_t size_q = 0;

    static uint8_t * p_key_u = m_uvwx_key_press_scan_str;
    static uint8_t size_u = 0;

    static uint8_t * p_key_y = m_yz_key_press_scan_str;
    static uint8_t size_y = 0;

    static uint8_t * p_key_ent = m_ent_key_press_scan_str;

    uint32_t err_code;

    switch (p_evt->nrf_csense_evt_type)
    {

        case NRF_CSENSE_BTN_EVT_PRESSED:
//            if (p_evt->p_instance == (&m_button8))
//            {
////                err_code = app_timer_stop(m_long_hold_id);
////                APP_ERROR_CHECK(err_code);
//
//                m_long_hold = false;
//                
//                err_code = app_timer_start(m_long_hold_id, LONG_HOLD, NULL);
//                APP_ERROR_CHECK(err_code);
//            }
            break;


        case NRF_CSENSE_BTN_EVT_RELEASED:

//            // *** Button 1 = 'abcd' | Send Key (Return) *** //
//            if (p_evt->p_instance == (&m_button1))   
//            {
//
//                if (m_change == true)
//                {
//                  
//                    if ((m_keyboard_increment == false) || (m_last_keyboard_input != 'a') || (size_a == 0))
//                    {
//                        p_key_a = m_abcd_key_press_scan_str;
//                        size_a  = 0;                    
//                        keys_send(1,p_key_a);
//                        p_key_a++;
//                        size_a++; 
//
//                        err_code = app_timer_stop(m_keyboard_input_id);
//                        APP_ERROR_CHECK(err_code);
//                    }
//                  
//                    else if (size_a <= MAX_KEYS_IN_ONE_REPORT-1)
//                    {                   
//                        keys_send(2,p_key_a);
//                        p_key_a++;
//                        p_key_a++;
//                        size_a++;
//                        size_a++;
//
//                        err_code = app_timer_stop(m_keyboard_input_id);
//                        APP_ERROR_CHECK(err_code);
//                    }
//                  
//                    else
//                    {
//                        keys_send(1,p_key_a);
//                        p_key_a = m_abcd_key_press_scan_str;
//                        size_a  = 0;
//                    }
//                
//                    m_keyboard_increment = true;
//                    m_last_keyboard_input = 'a';
//
//                    err_code = app_timer_start(m_keyboard_input_id, KEYBOARD_INPUT_INCREMENT, NULL);
//                    APP_ERROR_CHECK(err_code);
//                }    
//               
//                else
//                {
//                    keys_send(1, p_key_ent);  // Keyboard Return ('Enter')                
//                }
//
//                uint16_t * btn_cnt_1 = ((uint16_t *)p_evt->p_instance->p_context);
//                (*btn_cnt_1)++;
//                NRF_LOG_INFO("Button 1 touched %03d times.", (*btn_cnt_1));
//                
//              //bsp_board_led_on(BSP_BOARD_LED_1);
//            }

       
            // *** Button 2 = 'efgh' | Left Click *** //
            if (p_evt->p_instance == (&m_button2))   
            {

                if (m_change == true)
                {
                  
                    if ((m_keyboard_increment == false) || (m_last_keyboard_input != 'e') || (size_e == 0))
                    {
                        p_key_e = m_efgh_key_press_scan_str;
                        size_e  = 0;                    
                        keys_send(1,p_key_e);
                        p_key_e++;
                        size_e++; 

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else if (size_e <= MAX_KEYS_IN_ONE_REPORT-1)
                    {                   
                        keys_send(2,p_key_e);
                        p_key_e++;
                        p_key_e++;
                        size_e++;
                        size_e++;

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else
                    {
                        keys_send(1,p_key_e);
                        p_key_e = m_efgh_key_press_scan_str;
                        size_e  = 0;
                    }
                
                    m_keyboard_increment = true;
                    m_last_keyboard_input = 'e';

                    err_code = app_timer_start(m_keyboard_input_id, KEYBOARD_INPUT_INCREMENT, NULL);
                    APP_ERROR_CHECK(err_code);
                }
               
                else
                {
                    mouse_click_send(1,0,0);  // Mouse Left-Click
                    mouse_click_send(0,0,0);
                }

                uint16_t * btn_cnt_2 = ((uint16_t *)p_evt->p_instance->p_context);
                (*btn_cnt_2)++;
                NRF_LOG_INFO("Button 2 touched %03d times.", (*btn_cnt_2));
            }
            
            // *** Button 3 = 'ijkl' | Left Movement *** //
            if (p_evt->p_instance == (&m_button3)) 
            {

                if (m_change == true)
                {
                  
                    if ((m_keyboard_increment == false) || (m_last_keyboard_input != 'i') || (size_i == 0))
                    {
                        p_key_i = m_ijkl_key_press_scan_str;
                        size_i  = 0;                    
                        keys_send(1,p_key_i);
                        p_key_i++;
                        size_i++; 

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else if (size_i <= MAX_KEYS_IN_ONE_REPORT-1)
                    {                   
                        keys_send(2,p_key_i);
                        p_key_i++;
                        p_key_i++;
                        size_i++;
                        size_i++;

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else
                    {
                        keys_send(1,p_key_i);
                        p_key_i = m_ijkl_key_press_scan_str;
                        size_i  = 0;
                    }
                
                    m_keyboard_increment = true;
                    m_last_keyboard_input = 'i';

                    err_code = app_timer_start(m_keyboard_input_id, KEYBOARD_INPUT_INCREMENT, NULL);
                    APP_ERROR_CHECK(err_code);
                }
               
                else
                {
                    mouse_movement_send(-MOVEMENT_SPEED, 0);  // LEFT
                }

                uint16_t * btn_cnt_3 = ((uint16_t *)p_evt->p_instance->p_context);
                (*btn_cnt_3)++;
                NRF_LOG_INFO("Button 3 touched %03d times.", (*btn_cnt_3));
            }
            
         
            // *** Button 4 = 'mnop' | Up Movement *** //
            if (p_evt->p_instance == (&m_button4)) 
            {

                if (m_change == true)
                {

                    if ((m_keyboard_increment == false) || (m_last_keyboard_input != 'm') || (size_m == 0))
                    {
                        p_key_m = m_mnop_key_press_scan_str;
                        size_m  = 0;                    
                        keys_send(1,p_key_m);
                        p_key_m++;
                        size_m++; 

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else if (size_m <= MAX_KEYS_IN_ONE_REPORT-1)
                    {                   
                        keys_send(2,p_key_m);
                        p_key_m++;
                        p_key_m++;
                        size_m++;
                        size_m++;

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else
                    {
                        keys_send(1,p_key_m);
                        p_key_m = m_mnop_key_press_scan_str;
                        size_m  = 0;
                    }
                
                    m_keyboard_increment = true;
                    m_last_keyboard_input = 'm';

                    err_code = app_timer_start(m_keyboard_input_id, KEYBOARD_INPUT_INCREMENT, NULL);
                    APP_ERROR_CHECK(err_code);
                }
                

                else
                {
                    mouse_movement_send(0,-MOVEMENT_SPEED);  // UP      
                }
                
               
                uint16_t * btn_cnt_4 = ((uint16_t *)p_evt->p_instance->p_context);
                (*btn_cnt_4)++;
                NRF_LOG_INFO("Button 4 touched %03d times.", (*btn_cnt_4));
            }    
                   
            
            // *** Button 5 = 'qrst' | Down Movement *** //
            if (p_evt->p_instance == (&m_button5)) 
            {

                if (m_change == true)
                {
                  
                    if ((m_keyboard_increment == false) || (m_last_keyboard_input != 'q') || (size_q == 0))
                    {
                        p_key_q = m_qrst_key_press_scan_str;
                        size_q  = 0;                    
                        keys_send(1,p_key_q);
                        p_key_q++;
                        size_q++; 

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else if (size_q <= MAX_KEYS_IN_ONE_REPORT-1)
                    {                   
                        keys_send(2,p_key_q);
                        p_key_q++;
                        p_key_q++;
                        size_q++;
                        size_q++;

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else
                    {
                        keys_send(1,p_key_q);
                        p_key_q = m_qrst_key_press_scan_str;
                        size_q  = 0;
                    }
                
                    m_keyboard_increment = true;
                    m_last_keyboard_input = 'q';

                    err_code = app_timer_start(m_keyboard_input_id, KEYBOARD_INPUT_INCREMENT, NULL);
                    APP_ERROR_CHECK(err_code);
                }
               
                else
                {
                    mouse_movement_send(0,MOVEMENT_SPEED);  // DOWN
                }

                uint16_t * btn_cnt_5 = ((uint16_t *)p_evt->p_instance->p_context);
                (*btn_cnt_5)++;
                NRF_LOG_INFO("Button 5 touched %03d times.", (*btn_cnt_5));
            }    
            


            // *** Button 6 = 'uvwx' | Right Movement *** // 
            if (p_evt->p_instance == (&m_button6)) 
            {

                if (m_change == true)
                {
                  
                    if ((m_keyboard_increment == false) || (m_last_keyboard_input != 'u') || (size_u == 0))
                    {
                        p_key_u = m_uvwx_key_press_scan_str;
                        size_u  = 0;                    
                        keys_send(1,p_key_u);
                        p_key_u++;
                        size_u++; 

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else if (size_u <= MAX_KEYS_IN_ONE_REPORT-1)
                    {                   
                        keys_send(2,p_key_u);
                        p_key_u++;
                        p_key_u++;
                        size_u++;
                        size_u++;

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else
                    {
                        keys_send(1,p_key_u);
                        p_key_u = m_uvwx_key_press_scan_str;
                        size_u  = 0;
                    }
                
                    m_keyboard_increment = true;
                    m_last_keyboard_input = 'u';

                    err_code = app_timer_start(m_keyboard_input_id, KEYBOARD_INPUT_INCREMENT, NULL);
                    APP_ERROR_CHECK(err_code);
                }
               
                else
                {
                    mouse_movement_send(MOVEMENT_SPEED, 0);   // RIGHT
                }

                uint16_t * btn_cnt_6 = ((uint16_t *)p_evt->p_instance->p_context);
                (*btn_cnt_6)++;
                NRF_LOG_INFO("Button 6 touched %03d times.", (*btn_cnt_6));
            }            


            // *** Button 7 = 'yz' | Right Click *** //
            if (p_evt->p_instance == (&m_button7)) 
            {

                if (m_change == true)
                {
                  
                    if ((m_keyboard_increment == false) || (m_last_keyboard_input != 'y') || (size_y == 0))
                    {
                        p_key_y = m_yz_key_press_scan_str;
                        size_y  = 0;                    
                        keys_send(1,p_key_y);
                        p_key_y++;
                        size_y++; 

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else if (size_y <= MAX_KEYS_IN_ONE_REPORT-5)
                    {                   
                        keys_send(2,p_key_y);
                        p_key_y++;
                        p_key_y++;
                        size_y++;
                        size_y++;

                        err_code = app_timer_stop(m_keyboard_input_id);
                        APP_ERROR_CHECK(err_code);
                    }
                  
                    else
                    {
                        keys_send(1,p_key_y);
                        p_key_y = m_yz_key_press_scan_str;
                        size_y  = 0;
                    }
                
                    m_keyboard_increment = true;
                    m_last_keyboard_input = 'y';

                    err_code = app_timer_start(m_keyboard_input_id, KEYBOARD_INPUT_INCREMENT, NULL);
                    APP_ERROR_CHECK(err_code);
                }
               
                else
                {
                    mouse_click_send(2,0,0);    // Right Click
                    mouse_click_send(0,0,0);
                }

                uint16_t * btn_cnt_7 = ((uint16_t *)p_evt->p_instance->p_context);
                (*btn_cnt_7)++;
                NRF_LOG_INFO("Button 7 touched %03d times.", (*btn_cnt_7));
            }  



            // *** Button 8 = Delete / Caps Toggle (double click) / Change (long hold) *** //
          
//            if (p_evt->p_instance == (&m_button8))
//            {
//                
//                if (m_long_hold == true)
//                {
//                    m_change = !m_change;
//                    m_long_hold = false;
//                }
//
//                else if (m_double_click == true)
//                {
//                    err_code = app_timer_stop(m_double_click_id);
//                    APP_ERROR_CHECK(err_code);
//
//                    m_caps_on = !m_caps_on;
//                    m_double_click = false;
//                }
//
//                else
//                {                   
//                    err_code = app_timer_start(m_double_click_id, DOUBLE_CLICK, NULL);
//                    APP_ERROR_CHECK(err_code);
//
//                    m_double_click = true;
//
////                    uint8_t * p_key = m_del_key_press_scan_str;
////                    keys_send(1,p_key);
//                }
//
//                m_long_hold = false;
//
//                uint16_t * btn_cnt_8 = ((uint16_t *)p_evt->p_instance->p_context);
//                (*btn_cnt_8)++;                  
//                NRF_LOG_INFO("Button 8 touched %03d times.", (*btn_cnt_8));
//            }
//
//
//            else
//            {
//                //bsp_board_led_off(BSP_BOARD_LED_1);
//                break;
//            }

            break;
       
        
        default:
            NRF_LOG_WARNING("Unknown event.");
            break;

    }
}



/**
 * @brief Function for starting Capacitive Sensor High module.
 *
 * Function enables four buttons.
 */
static void csense_start(void)
{
    ret_code_t err_code;

    static uint16_t touched_counter_1 = 0;
    static uint16_t touched_counter_2 = 0;
    static uint16_t touched_counter_3 = 0;
    static uint16_t touched_counter_4 = 0;
    static uint16_t touched_counter_5 = 0;
    static uint16_t touched_counter_6 = 0;
    static uint16_t touched_counter_7 = 0;
    static uint16_t touched_counter_8 = 0;

    err_code = nrf_csense_init(nrf_csense_handler, APP_TIMER_TICKS_TIMEOUT);
    APP_ERROR_CHECK(err_code);

//    nrf_csense_instance_context_set(&m_button1, (void*)&touched_counter_1);
    nrf_csense_instance_context_set(&m_button2, (void*)&touched_counter_2);
    nrf_csense_instance_context_set(&m_button3, (void*)&touched_counter_3);
    nrf_csense_instance_context_set(&m_button4, (void*)&touched_counter_4);
    nrf_csense_instance_context_set(&m_button5, (void*)&touched_counter_5);
    nrf_csense_instance_context_set(&m_button6, (void*)&touched_counter_6);
//    nrf_csense_instance_context_set(&m_button7, (void*)&touched_counter_7);
//    nrf_csense_instance_context_set(&m_button8, (void*)&touched_counter_8);

        
//    err_code = nrf_csense_add(&m_button1);
//    APP_ERROR_CHECK(err_code);

    err_code = nrf_csense_add(&m_button2);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_csense_add(&m_button3);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_csense_add(&m_button4);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_csense_add(&m_button5);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_csense_add(&m_button6);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_csense_add(&m_button7);
//    APP_ERROR_CHECK(err_code);
    
//    err_code = nrf_csense_add(&m_button8);
//    APP_ERROR_CHECK(err_code);
   
}




/**@brief Function for application main entry.
 */
int main(void)
{
    //bool erase_bonds;     // Default value is false
    bool erase_bonds = true;  //*****

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
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
    NRF_LOG_INFO("HID Mouse with capacitive sensing combined example.");
    timers_start();

    advertising_start(erase_bonds);

    csense_start();

    // Configure BUTTON0 with SENSE enabled 
    nrf_gpio_cfg_sense_input(BUTTON_1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);    //***** BUTTON_1 defined for pin 13 in custom_board.h file ; pin pulled up and triggered by GND

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }

    while (1)
    {
        __WFI();
        NRF_LOG_FLUSH();
    }


/* Main application entry function from csense */

/*    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = clock_config();
    APP_ERROR_CHECK(err_code);
    
    csense_start();

    while (1)
    {
        __WFI();
        NRF_LOG_FLUSH();
    }

*/

}


/**
 * @}
 */
