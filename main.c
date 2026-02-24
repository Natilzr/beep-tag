
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"

#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

#include "ble_gap.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#include "nrf_delay.h" // For nrf_delay_ms()
#include "adv_h.h"
#include "adc.h"

#include "nrf_drv_pwm.h"
//#include "nrfx_pwm.h"
#include "app_util_platform.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
//#include "nrfx_timer.h"
#include "nrf_drv_timer.h"


#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_0  //BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

//#define DEVICE_NAME                     "Nordic_Blinky"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#ifdef  NRF52_10
      #ifdef  V1
        #define BUTTON_PIN                      14
      #else
        #define BUTTON_PIN                      9
      #endif
#else
  #define BUTTON_PIN                     4 // 5for buzzer
#endif

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#if WLI
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */
                                          




              
                                          

#define DYNAMIC_MANUF_DATA_LEN   31
/*      6H      code change    */ 
#define UPDATE_INTERVAL_MINUTES   120
                                          
#define UPDATE_INTERVAL_TICKS    APP_TIMER_TICKS(UPDATE_INTERVAL_MINUTES * 60 * 1000)
/*      10M      code change    */                                           
#define UPDATE_INTERVAL_MINUTES_TEST  5
                                          
#define UPDATE_INTERVAL_TICKS_TEST    APP_TIMER_TICKS(UPDATE_INTERVAL_MINUTES_TEST * 60 * 1000)
  
/*      1M       ADC interval   */  
#define UPDATE_INTERVAL_MINUTES_ADC  10
#define UPDATE_INTERVAL_TICKS_ADC    APP_TIMER_TICKS(UPDATE_INTERVAL_MINUTES_ADC * 60 * 1000)    

#define UPDATE_INTERVAL_TICKS_SOUND    APP_TIMER_TICKS(300) // 1 sec   
                                          
#define UPDATE_INTERVAL_MINUTES_ADC_TEST  2
#define UPDATE_INTERVAL_TICKS_ADC_TEST    APP_TIMER_TICKS(UPDATE_INTERVAL_MINUTES_ADC_TEST * 60 * 1000) 
 /*     10M   disapear          */
#define UPDATE_INTERVAL_MINUTES_DISS  10
#define UPDATE_INTERVAL_TICKS_DISS    APP_TIMER_TICKS(UPDATE_INTERVAL_MINUTES_DISS * 60 * 1000)                                             
                                          

#define UPDATE_RESET_INTERVAL    APP_TIMER_TICKS(500)                                          
                                          
                                          
APP_TIMER_DEF(m_adv_update_timer);    

//APP_TIMER_DEF(m_sound_update_timer);    


//APP_TIMER_DEF(m_button_timer_id);

APP_TIMER_DEF(ADC_Update_timer);



APP_TIMER_DEF(Reset_timer);     

#define WAKEUP_BUTTON_PIN 4//was 5
#define HOLD_TIME_MS 2000


void DISS_timer_handler(void *p_context);

                                          
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);

static void Reset_timer_handler(void *p_context);
void adv_update_timer_handler(void *p_context);
void sound_timer_handler(void *p_context);

static void notify_write_success(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t status_code);

void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
void GetDataUUID(uint8_t *new_manuf_data);
void saadc_sample(void);



 volatile bool battery_ready = false;

 volatile bool  adv_upd = false;
    
 
//uint8_t RecNumG;    
    
#if sound2
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
#endif
#define PIEZO_PIN NRF_GPIO_PIN_MAP(0,5) 

#if 0
 static uint8_t m_beacon_info1[APP_BEACON_INFO_LENGTH1]=  
 {
   //APP_ADV_DATA_LENGTH1, // 1e(30)
//   APP_DEVICE_TYPE1,      //   ff 
    APP_BEACON_UUID1

 };  
#endif
#endif


#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */


uint8_t flash_ready = true;
uint8_t StorageState;
uint8_t RecCounter;
uint32_t MemAdr;
uint32_t NameAdr;
uint8_t NumCnt;


uint32_t MemReadAdr;
uint8_t NumRec = 0;
uint8_t ResetFlag = 0;
uint8_t BattPwr_prev,BattPwr = 0;
uint8_t new_batt_val;
uint8_t flash_empty;

uint8_t SRon;
uint8_t sound_cnt = 0;
uint8_t sound_task = 0;
uint8_t sound_cnt2 = 0;
//static bool button_hold_confirmed = false;


/**@brief Struct that contains pointers to the encoded advertising data. */
ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
    
    bsp_board_led_on(LEDBUTTON_LED);
}
#if sound2
static void buzzer_init(void)
{
    nrf_gpio_cfg_output(5);
    nrf_gpio_cfg_output(12); 
     nrf_gpio_pin_write(5,0);
     nrf_gpio_pin_write(12,0);
}
#define USED_PWM(idx) (1UL << idx)
//static uint8_t m_used = 0;

#endif

#if 0
static void demo1(void)
{
#define PWM_TOP 700   // 1 MHz / 1000 = 1 kHz

  #define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
  
static nrf_pwm_values_individual_t seq_values =
{

  .channel_0 = PWM_TOP / 4,      // 25%
  .channel_1 = 3 * PWM_TOP / 4, // 75%
    .channel_2 = 0,
    .channel_3 = 0
};

nrf_pwm_sequence_t const seq =
{
    .values.p_individual = &seq_values,
    .length = NRF_PWM_VALUES_LENGTH(seq_values),
};

nrf_drv_pwm_config_t const config0 =
{
    .output_pins =
    {
        5,
        12 ,//| NRF_DRV_PWM_PIN_INVERTED,
        NRF_DRV_PWM_PIN_NOT_USED,
        NRF_DRV_PWM_PIN_NOT_USED
    },
    .base_clock = NRF_PWM_CLK_1MHz,
    .top_value  = PWM_TOP,
    .load_mode  = NRF_PWM_LOAD_INDIVIDUAL,
};

nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);

// wait 1 second
nrf_delay_ms(1000);

// stop PWM
nrf_drv_pwm_stop(&m_pwm0, true);  // true = stop immediately

}
#endif
#if sound2
uint16_t pwm_top_for_freq(uint32_t freq_hz)
{
    return (uint16_t)(1000000 / freq_hz); // base_clock = 1 MHz
}

void play_note(uint32_t freq_hz, uint16_t duration_ms)
{
    uint16_t top = pwm_top_for_freq(freq_hz);

    // Set new TOP
    //nrf_drv_pwm_stop(&m_pwm0, true);
    nrf_drv_pwm_uninit(&m_pwm0);
nrf_pwm_sequence_t seq;
static nrf_pwm_values_individual_t seq_values[1];

seq_values[0].channel_0 = top / 4;      // 25%
seq_values[0].channel_1 = 3 * top / 4; // 75%
seq_values[0].channel_2 = 3 * 0;
seq_values[0].channel_3 = 3 * 0;

seq.values.p_individual = seq_values;
seq.length = NRF_PWM_VALUES_LENGTH(seq_values);
seq.end_delay = 0;
seq.repeats = 0;
 

    nrf_drv_pwm_config_t const config0 =
{
    .output_pins =
    {
        5,
        12 ,//| NRF_DRV_PWM_PIN_INVERTED,
        NRF_DRV_PWM_PIN_NOT_USED,
        NRF_DRV_PWM_PIN_NOT_USED
    },
    .base_clock = NRF_PWM_CLK_1MHz,
    .top_value  = top,
    .load_mode  = NRF_PWM_LOAD_INDIVIDUAL,
};

nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);

    nrf_delay_ms(duration_ms);

    nrf_drv_pwm_stop(&m_pwm0, true);
}


void play_note_start(uint32_t freq_hz, uint16_t duration_ms)
{
    uint16_t top = pwm_top_for_freq(freq_hz);

    // Set new TOP
    //nrf_drv_pwm_stop(&m_pwm0, true);
    nrf_drv_pwm_uninit(&m_pwm0);
nrf_pwm_sequence_t seq;
static nrf_pwm_values_individual_t seq_values[1];

seq_values[0].channel_0 = top / 4;      // 25%
seq_values[0].channel_1 = 3 * top / 4; // 75%
seq_values[0].channel_2 = 3 * 0;
seq_values[0].channel_3 = 3 * 0;

seq.values.p_individual = seq_values;
seq.length = NRF_PWM_VALUES_LENGTH(seq_values);
seq.end_delay = 0;
seq.repeats = 0;
 

    nrf_drv_pwm_config_t const config0 =
{
    .output_pins =
    {
        5,
        12 ,//| NRF_DRV_PWM_PIN_INVERTED,
        NRF_DRV_PWM_PIN_NOT_USED,
        NRF_DRV_PWM_PIN_NOT_USED
    },
    .base_clock = NRF_PWM_CLK_1MHz,
    .top_value  = top,
    .load_mode  = NRF_PWM_LOAD_INDIVIDUAL,
};

nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);


}

void play_note_stop(uint32_t freq_hz, uint16_t duration_ms)
{

    nrf_drv_pwm_uninit(&m_pwm0);

}
#endif


#define PIEZO_PIN1  5
#define PIEZO_PIN2  12

static volatile bool pin1_state = false;

// Timer2 interrupt handler
void TIMER2_IRQHandler(void)
{
    if (NRF_TIMER2->EVENTS_COMPARE[0])
    {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;  // Clear event
        
        pin1_state = !pin1_state;
        
        if (pin1_state) {
            nrf_gpio_pin_set(PIEZO_PIN1);
            nrf_gpio_pin_clear(PIEZO_PIN2);
        } else {
            nrf_gpio_pin_clear(PIEZO_PIN1);
            nrf_gpio_pin_set(PIEZO_PIN2);
        }
    }
}

void piezo_init(void)
{
    // Configure pins with HIGH DRIVE strength
    nrf_gpio_cfg(PIEZO_PIN1,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1,
                 NRF_GPIO_PIN_NOSENSE);
    
    nrf_gpio_cfg(PIEZO_PIN2,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1,
                 NRF_GPIO_PIN_NOSENSE);
    
    // Set initial states - complementary
    nrf_gpio_pin_clear(PIEZO_PIN1);
    nrf_gpio_pin_clear(PIEZO_PIN2);
    pin1_state = false;
    
    // Stop timer if running
    NRF_TIMER2->TASKS_STOP = 1;
    NRF_TIMER2->TASKS_CLEAR = 1;
    
    // Configure timer
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER2->PRESCALER = 0;  // 16MHz
    
    // Set compare value for 3kHz (toggle at 6kHz)
    // 16MHz / 6000 = 2667
    NRF_TIMER2->CC[0] = 2667;
    
    // Enable interrupt on compare[0]
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    
    // Enable shortcut to clear timer on compare
    NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    
    // Enable TIMER2 interrupt in NVIC
    NVIC_SetPriority(TIMER2_IRQn, 6);
    NVIC_EnableIRQ(TIMER2_IRQn);
}

void piezo_start(void)
{
    NRF_TIMER2->TASKS_START = 1;
}

void piezo_stop(void)
{
    NRF_TIMER2->TASKS_STOP = 1;
    nrf_gpio_pin_clear(PIEZO_PIN1);
    nrf_gpio_pin_clear(PIEZO_PIN2);
}

void play_note(uint16_t freq, uint16_t times)
{
  uint16_t pres;
//  if(sound_task) pres = 16000000/(freq);
//  else
  pres = 16000000/(freq*2);
  NRF_TIMER2->CC[0] = pres;
  piezo_start();
  nrf_delay_ms(times);
  piezo_stop();
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    // Create the timer
err_code = app_timer_create(&m_adv_update_timer,
                            APP_TIMER_MODE_REPEATED,
                            adv_update_timer_handler);
APP_ERROR_CHECK(err_code);

    // Create the timer
err_code = app_timer_create(&ADC_Update_timer,
                            APP_TIMER_MODE_REPEATED,
                            adc_timer_handler);
APP_ERROR_CHECK(err_code);

//err_code = app_timer_create(&m_sound_update_timer,
//                            APP_TIMER_MODE_REPEATED,
//                            sound_timer_handler);
//APP_ERROR_CHECK(err_code);


/*
    // Create the timer
err_code = app_timer_create(&DISS_Update_timer,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            DISS_timer_handler);
APP_ERROR_CHECK(err_code);
*/
err_code = app_timer_create(&Reset_timer,
                            APP_TIMER_MODE_SINGLE_SHOT,
                            Reset_timer_handler);
APP_ERROR_CHECK(err_code);

// Start the timer
/*
err_code = app_timer_start(m_adv_update_timer, UPDATE_INTERVAL_TICKS, NULL);
err_code = app_timer_start(ADC_Update_timer, UPDATE_INTERVAL_TICKS_ADC, NULL);
APP_ERROR_CHECK(err_code);
*/
}



static void app_timers_start(void)
{
  // Start the timer
  ret_code_t err_code;
err_code = app_timer_start(m_adv_update_timer, UPDATE_INTERVAL_TICKS, NULL);
err_code = app_timer_start(ADC_Update_timer, UPDATE_INTERVAL_TICKS_ADC, NULL);
//err_code = app_timer_start(m_sound_update_timer, UPDATE_INTERVAL_TICKS_SOUND, NULL);

APP_ERROR_CHECK(err_code);
}
#if 0
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
    char* p = (char*)0x21d00;
    char DEVICE_NAME[18];
    strncpy(DEVICE_NAME,p,sizeof(DEVICE_NAME)-1);
    DEVICE_NAME[sizeof(DEVICE_NAME)-1] = '\0';

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

#endif
/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED);
 //       NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED);
  //      NRF_LOG_INFO("Received LED OFF!");
    }
}

static void ppk1_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t const * data, uint16_t len)
{
  ret_code_t rc;
  static uint8_t RecNum[8];
      uint32_t              err_code;
  if(len == 4)
  {
    if ((data[0] == 'W') && (data[1] == 'L'))
    {
      switch(data[2])
      {
      case '0':
           StorageState = NoMem;
          break;
      case '1':
          StorageState = Erasing;
          rc = nrf_fstorage_erase(&fstorage, 0x25000, 1, NULL);  // Erase 1 page
          APP_ERROR_CHECK(rc);  // Always check return codes
          MemAdr = 0x25000;
          MemReadAdr = 0x25000;
          NameAdr =0x25d00;
          notify_write_success(conn_handle, p_lbs, 0x01);  // 0x01 = success
          break;
      case '2':
        StorageState = SaveRec;
          notify_write_success(conn_handle, p_lbs, 0x02);  // 0x01 = success
          break;
      case '3':
        RecNum[0] = NumCnt/2;
        NumRec = RecNum[0];
        rc = nrf_fstorage_write(&fstorage,0x25e00, RecNum,sizeof(uint64_t), NULL);
        APP_ERROR_CHECK(rc);  // Always check return codes
        notify_write_success(conn_handle, p_lbs, 0x03);  // 0x01 = success
        StorageState = NoMem;
        err_code = app_timer_start(Reset_timer, UPDATE_RESET_INTERVAL, NULL);
        APP_ERROR_CHECK(err_code);
        bsp_board_led_off(LEDBUTTON_LED);
          break;
      case '4':
        StorageState = SaveName;
          break;
      default:
        break;
      }
             
    }
  }
  


  

}

static void ppk_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t const * data, uint16_t len)
//static void ppk_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
 // uint32_t num = 0x12345678;
  ret_code_t rc;
  #pragma data_alignment=4
static uint8_t aligned_data[16];
memcpy(aligned_data, data, 16);
static uint8_t RecNum[8];
 //  uint32_t data_to_write = (led_state) ? 0xAABBCCDD : 0x11223344;
  switch (StorageState)
  {
  case NoMem:
    break;
  case Erasing:
    break;
  case SaveRec:
    if(len == 16)
    {
   //    rc = nrf_fstorage_write(&fstorage, MemAdr, data, sizeof(uint64_t), NULL);
 
      rc = nrf_fstorage_write(&fstorage, MemAdr, aligned_data,len, NULL);
      APP_ERROR_CHECK(rc);  // Always check return codes
      MemAdr += 16;
      NumCnt ++;
      notify_write_success(conn_handle, p_lbs, 0x02);  // 0x01 = success
 //     NRF_LOG_INFO("wrote ok");
    }
        break;
  case SaveRecNum:
        RecNum[0] = NumCnt/2;
        NumRec = RecNum[0];
        rc = nrf_fstorage_write(&fstorage,0x25e00, RecNum,sizeof(uint64_t), NULL);
        APP_ERROR_CHECK(rc);  // Always check return codes
 //       NRF_LOG_INFO("wrote RecNum");
        break;
  case SaveName:
        rc = nrf_fstorage_write(&fstorage, 0x25d00, aligned_data,sizeof(uint64_t), NULL);
        APP_ERROR_CHECK(rc);  // Always check return codes
 //       NRF_LOG_INFO("wrote ok");
        NameAdr += 8;
        break;
  default:
    break;
  }
    

  


}

static void version_read_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t const * data, uint16_t len)
{

    
}

static void buzz_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t const * data, uint16_t len)
{
//demo1();

//  ret_code_t err_code;
//  err_code = app_timer_start(m_sound_update_timer, UPDATE_INTERVAL_TICKS_SOUND, NULL);

//APP_ERROR_CHECK(err_code);
    sound_task = 1;
}



// Add this helper function
static void notify_write_success(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t status_code)
{
    uint8_t notify_data[] = { status_code };  // You can define status codes: 0x01 = OK, etc.
    uint16_t len = sizeof(notify_data);

    ble_gatts_hvx_params_t hvx_params = {0};
    hvx_params.handle = p_lbs->ppk2_char_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.p_data = notify_data;
    hvx_params.p_len  = &len;

    ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
    if (err_code != NRF_SUCCESS) {
        // Optional: log error or ignore
        // NRF_LOG_WARNING("Notification failed: 0x%x", err_code);
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
  

    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.led_write_handler =    led_write_handler;
    init.ppk_write_handler =    ppk_write_handler;
    init.ppk1_write_handler =   ppk1_write_handler;
    init.version_read_handler = version_read_handler;
    init.buzz_write_handler =    buzz_write_handler;
    
    
    
    
    
    

    
    

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
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
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

 //   bsp_board_led_on(ADVERTISING_LED);
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
 //           NRF_LOG_INFO("Connected");
            bsp_board_led_on(CONNECTED_LED);
//            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
 //           NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
  //          NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

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


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
#if 0
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
 //   ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            NRF_LOG_INFO("push");
 /*
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
*/
        if (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN) == 0) {
        app_timer_start(m_button_timer_id, APP_TIMER_TICKS(HOLD_TIME_MS), NULL);
    } else {
        app_timer_stop(m_button_timer_id);
    }
    }
    
    

    
    
}

void button_hold_timer_handler(void *p_context) {
    if (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN) == 0) {
        button_hold_confirmed = true;
    }
}


void wait_for_button_hold(void) {
    app_timer_create(&m_button_timer_id, APP_TIMER_MODE_SINGLE_SHOT, button_hold_timer_handler);
//    button_init();
    app_button_enable();

    NRF_LOG_INFO("Sleeping until button held for 2s...");

    while (!button_hold_confirmed) {
        sd_app_evt_wait();  // System ON sleep mode, SoftDevice-safe
    }
app_button_disable();
    NRF_LOG_INFO("Button held long enough. Starting app...");
}
#endif

void wait_for_button_hold(void) {
    // Configure button with SENSE feature
    nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

    NRF_LOG_INFO("Sleeping until button held for 2s...");

    uint32_t start_ticks = app_timer_cnt_get();
    bool button_still_held = false;

    while (!button_still_held) {
        sd_app_evt_wait();  // Sleep until GPIO sense wakes us

        // Wait for 2s hold
        if (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
            nrf_delay_ms(2000);
            if (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
                button_still_held = true;
            }
        }
    }

    NRF_LOG_INFO("Button held long enough. Starting app...");
}


/* GPIOTE event is used only to start periodic timer when first button is activated. */
static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}


/**@brief Function for initializing the button handler module.
 */

static void buttons_init(void)
{
  
  ret_code_t err_code;

      if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);  // or HITOLO/LOTOHI as needed
config.pull = NRF_GPIO_PIN_PULLUP;

 err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &config, gpiote_event_handler);
APP_ERROR_CHECK(err_code);
nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
    
}


#if 0
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
#endif

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}



void save_param(void)
{

    // ret_code_t rc;
    MemReadAdr = 0x25000;
    MemAdr = 0x25000;
    BattPwr = 0x24;
    NumRec = 0;
    StorageState = NoMem;
    ret_code_t err_code = nrf_fstorage_init(&fstorage, &nrf_fstorage_sd, NULL);
    APP_ERROR_CHECK(err_code);



}

static void Reset_timer_handler(void *p_context)
{
  sd_nvic_SystemReset();   // If using SoftDevice
}





void blink_led(uint8_t led_pin, uint8_t times, uint32_t delay_ms) {
    for (uint8_t i = 0; i < times; i++) {
        nrf_gpio_pin_toggle(led_pin); // Toggle LED ON
        nrf_delay_ms(delay_ms);
        nrf_gpio_pin_toggle(led_pin); // Toggle LED OFF
        nrf_delay_ms(delay_ms);
    }
}

bool is_button_pressed(void) {
    return nrf_gpio_pin_read(BUTTON_PIN) == 0;
}

//void sound_timer_stop(void)
//{
//    app_timer_stop(m_sound_update_timer);
//}

void wait_ms(uint32_t ms) {
    nrf_delay_ms(ms);  // Not power-efficient, but fine for short waits
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
  SRon = 1;
  flash_empty = 0;
 //   log_init();
    leds_init();
//    buzzer_init();
    timers_init();
    buttons_init();
    power_management_init();


       // Simulate FDS logic — assume this is first boot
      // Simulate FDS logic — assume this is first boot
 
    ble_stack_init();
#if 0
    sd_app_evt_wait();  // System ON sleep mode, SoftDevice-safe

     while (true) 
     {
       if (is_button_pressed()) 
       {
        wait_ms(2000);
        if (is_button_pressed()) 
        {
            // Long press confirmed
            break;
            // Do your action
        } else 
        {
    // Optional: clear any flags
    nrf_pwr_mgmt_run();
    sd_app_evt_wait();  // Go back to System ON sleep
        }
      }
    }
#endif
    piezo_init();


play_note(3000,30);
play_note(4500,30); 
play_note(6500,50);
play_note(3000,30);
play_note(5000,30); 
play_note(6500,50);
play_note(3000,30);
play_note(4500,30); 
play_note(3000,50);
play_note(5000,30);
play_note(6500,50); 
play_note(5000,50);

        blink_led(LED_1, 5, 200); // Adjust LED_1 and delay as needed





    app_timers_start();

//    set_custom_ble_address();
 //   gap_params_init();
        NRF_LOG_INFO("wake");
#ifdef  ADC_W
      //  saadc_init();
      
#endif
    gatt_init();
    services_init();
 //   advertising_init();
    conn_params_init();
    save_param();
    GetName(new_name);
    wait_ms(1000);
       NRF_LOG_INFO("NRF started.");
//advertising_update_runtime(new_manuf_data,SRon);
       adv_upd = true;
    // Start execution.
    // Enter main loop.
    for (;;)
    {
      if(sound_task)
      {
        switch(sound_cnt)
        {     
            //sound_timer_stop();
            case 0: //3000
            case 7:
            case 20:
              nrf_gpio_pin_toggle(LED_1); // Toggle LED ON
              play_note(3000,30);
              break;
            case 1:  //4500
            case 6:
            case 19:
            case 21:
              nrf_gpio_pin_toggle(LED_1); // Toggle LED ON
              play_note(4500,30);
              break;
            case 2:     //6500
            case 4:
            case 9:
            case 11:
            case 14:
            case 16:
            case 18:
              nrf_gpio_pin_toggle(LED_1); // Toggle LED ON
              play_note(6500,50);
              break;
            case 3:     //5000
            case 5:
            case 8:
            case 10:
            case 13:
            case 15:
            case 17:
              nrf_gpio_pin_toggle(LED_1); // Toggle LED ON
              play_note(5000,30);
              break;
            case 12:
              play_note(500,100);
              break;
            default:
              sound_cnt = 0;
              nrf_gpio_pin_clear(LED_1);
              sound_task = 0;
              break;
            }
            sound_cnt++;

      }
              if ((battery_ready) || (adv_upd))
              {
                if (battery_ready)
                {
                  battery_ready = false;
                }
                if (adv_upd)
                {
                  adv_upd = false;
                }
                advertising_update_runtime(new_manuf_data, SRon);
                
              }
              
              idle_state_handle();
            
            }
}



