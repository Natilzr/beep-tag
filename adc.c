
#include "adc.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "adv_h.h"



extern uint8_t new_batt_val;

extern uint8_t SRon;
extern  volatile bool battery_ready;
extern  volatile bool adv_upd;

uint16_t prev_battery_voltage;

uint16_t mv;


#define SAADC_SAMPLES_IN_BUFFER 3


#define TH_VAL          20

//static nrf_saadc_value_t     m_buffer_pool[SAADC_SAMPLES_IN_BUFFER];   

static nrf_saadc_value_t adc_buffer;

#ifdef  ADC_W
void saadc_sample(void) {
    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}
#endif




void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    // Only signal readiness when the code actually changes
    // AND the voltage moved by at least a small amount.
    uint32_t delta;
    uint8_t new_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        // Grab the raw result
        int16_t raw = p_event->data.done.p_buffer[0];
        // Convert to mV (assuming 3.6 V reference, 10-bit resolution)
        mv = ((uint32_t)raw * 3600) / 1023;
//        NRF_LOG_INFO("Battery: %d mV", mv);
          if (mv > 3000)//full
          {
            new_code = 0x24;
          }
          else if (mv > 2900)//medium
          {
            new_code = 0x25;
          }
          else if (mv > 2700)//low
          {
            new_code = 0x26;
          }    
          else//critical
          {
            new_code = 0x27;
          }
          if (mv > prev_battery_voltage) 
          {
            delta = mv - prev_battery_voltage;
          } 
          else 
          {
            delta = prev_battery_voltage - mv;
          }

          if (new_code != BattPwr && delta >= TH_VAL)
          {
            BattPwr_prev = BattPwr;
            BattPwr = new_code;
            battery_ready = true;
            prev_battery_voltage = mv; // treat this as previously reported battery voltage, not just the last reading.
          }
        // Release SAADC to save power
        nrf_drv_saadc_uninit();
    }
}


#if 0
void saadc_init(void) {
    ret_code_t err_code;
#if ALEX_H
   // 1) Pick a driver config that enables oversampling:
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_16X;   // hardware averages 16 conversions
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    saadc_config.low_power_mode = false;                  // disable LP mode if you want faster sampling
    // Init SAADC
//    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    // Configure channel to read VDD/4 (internal battery voltage)
    nrf_saadc_channel_config_t 
      channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    // 2) Configure your channel with .burst = true
    channel_config.burst = NRF_SAADC_BURST_ENABLED;  // take all oversample conversions back-to-back
    
    // Init one channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
    
        // ? Allocate a buffer before sampling
  //  err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool, SAADC_SAMPLES_IN_BUFFER);
        err_code = nrf_drv_saadc_buffer_convert(&m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
#endif
   // Init SAADC
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.low_power_mode=true;
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    // Configure channel to read VDD/4 (internal battery voltage)
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    // Init one channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
    
        // ? Allocate a buffer before sampling
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool, SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}
#endif
  






   

static void saadc_sample_once(void)
{
    ret_code_t err;

    // Init driver
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    err = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err);

    // Init channel
    nrf_saadc_channel_config_t channel_config =
//        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err);

    // Provide buffer
    err = nrf_drv_saadc_buffer_convert(&adc_buffer, 1);
    APP_ERROR_CHECK(err);

    // Trigger sample
    err = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err);
}

void adc_timer_handler(void *p_context)
{
    // Example: Update name and manufacturer data dynamically
#ifdef  ADC_W
    //saadc_sample();
     saadc_sample_once();   // init ? sample ? result comes in callback ? uninit
    // Save, log, or transmit mv...
#endif

  if(new_batt_val)
  {
    new_batt_val = 0;
    adv_upd = true;
 //   advertising_update_runtime(new_manuf_data, SRon);
    NRF_LOG_INFO("new_adc");
  }
  else
  {
    NRF_LOG_INFO("adc");
  }
}