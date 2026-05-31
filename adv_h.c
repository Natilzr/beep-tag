#include <stdint.h>
#include "app_error.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "ble_gap.h"
#include "adv_h.h"
#include "adc.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrf_delay.h" // For nrf_delay_ms()
uint8_t new_manuf_data[APP_BEACON_INFO_LENGTH1] = { 0 }; // Just incrementing for example
uint8_t new_name[20]__attribute__((aligned(4)));

//extern app_timer_id_t m_sound_update_timer;
extern void play_note_start(uint32_t freq_hz, uint16_t duration_ms);
extern void play_note_stop(uint32_t freq_hz, uint16_t duration_ms);
extern void play_note(uint16_t freq, uint16_t times);
void piezo_init(void);
//static uint8_t counter = 0;
extern uint8_t SRon;
extern uint8_t flash_empty;
extern uint8_t sound_cnt;
extern uint8_t sound_task;

extern  volatile bool  adv_upd;
extern void sound_timer_stop(void);

//extern uint8_t RecNumG; 

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    .evt_handler = fstorage_evt_handler,
    .start_addr  = 0x25000,
    .end_addr    = 0x26000,
};



static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
//        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
 //         flash_ready = true;
 //           NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
  //                       p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
 //         flash_ready = true;
 //           NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
 //                        p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}



void adv_update_timer_handler(void *p_context)
{
  // Advance index/address first, then advertise.
    const uint32_t base = 0x25000;

    uint8_t idx;
    idx = (uint8_t)((MemReadAdr - base) / 32);
    // Example: Update name and manufacturer data dynamically
    if (NumRec == 0) 
    { //on the off-chance NumRec hasn�t been initialized or set properly
        MemReadAdr = base;
    }
    else if (idx >= (NumRec - 1)) 
    {
      MemReadAdr = base;    // wrap to first record
    } else 
    {
      MemReadAdr += 32;     // advance to next record
    }

    
  

 //   snprintf((char *)new_name, sizeof(new_name), "NODE_%02d", counter);
    if(flash_empty == 0)
    {
#if 0
          SRon = 0;
#endif
    }

 //   advertising_update_runtime(new_manuf_data, SRon);
    adv_upd = 1;
 //   MemReadAdr += 32;
}


 #define LED_1          20
#if 0
void sound_timer_handler(void *p_context)
{
  sound_task = 1;
#if 0
    switch(sound_cnt)
    {
    case 0:
    case 2:
    case 4:
    case 6:
    case 8:
    case 10:
    case 12:
    play_note(6000,50);
    nrf_gpio_pin_toggle(LED_1); // Toggle LED ON
    nrf_delay_ms(10);
      break;
    case 1:
    case 3:
    case 5:
    case 7:
    case 9:
    case 11:
   // play_note_start(1000, 100);
    nrf_gpio_pin_toggle(LED_1); // Toggle LED ON
    nrf_delay_ms(10);
      break;
    case 13:
      nrf_gpio_pin_clear(LED_1);
  //    play_note_stop(1000, 100);
      break;
    default:
      sound_cnt = 0;
      
      sound_timer_stop();
      break;
    }
 //   advertising_update_runtime(new_manuf_data, SRon);
    sound_cnt++;
 //   MemReadAdr += 32;
#endif
}
#endif


volatile int iCntr = 0;
void AppErrorCheck(ret_code_t errCode)
{
  if (NRF_SUCCESS != errCode)
  {
    iCntr++;
    APP_ERROR_CHECK(errCode);
  }
}


void advertising_update_runtime(uint8_t *new_manuf_data,uint8_t Son)
{
    ret_code_t err_code;
    ret_code_t rc;
    // Make sure `fstorage` is initialized earlier in your code
    rc = nrf_fstorage_read(&fstorage, 0x25E00, &NumRec, sizeof(uint8_t));
    AppErrorCheck(rc);
if (m_advertising_running) {
    // Safe to stop or update advertising
    NRF_LOG_INFO("running");
    // Stop current advertising (safe even if not running)
    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        AppErrorCheck(err_code);
    }
}
    GetDataUUID(new_manuf_data);
    // ==== Set GAP device name ====
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    if(new_name[0]==0x57)
    {
      err_code = sd_ble_gap_device_name_set(&sec_mode, new_name, DYNAMIC_NAME_LEN);
      AppErrorCheck(err_code);
    }
  //  NRF_LOG_INFO("nam");
    // ==== Build advertising data ====
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    memset(&srdata, 0, sizeof(srdata));
    

    ble_advdata_manuf_data_t manuf_data;

    memset(&advdata, 0, sizeof(advdata));
    memset(&manuf_data, 0, sizeof(manuf_data));

    manuf_data.company_identifier = APP_COMPANY_IDENTIFIER1;
    manuf_data.data.p_data = (uint8_t *)new_manuf_data;
    manuf_data.data.size   = APP_BEACON_INFO_LENGTH1;

    advdata.name_type = BLE_ADVDATA_NO_NAME; // No name in advertising packet
    advdata.flags     = 0;//BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data = &manuf_data;

 //     srdata.name_type = BLE_ADVDATA_FULL_NAME; // GAP name will be pulled here
  //    srdata.include_appearance = true;
    

    // Encode advertisement packet
  //  m_adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    AppErrorCheck(err_code);
    NRF_LOG_INFO("ENCODE2");
    // Encode scan response packet

if (Son == 1)
{
    srdata.name_type = BLE_ADVDATA_FULL_NAME;
    srdata.include_appearance = true;

    m_adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    AppErrorCheck(err_code);

 //   adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
}
else
{
    // Disable scan response
    m_adv_data.scan_rsp_data.p_data = NULL;
    m_adv_data.scan_rsp_data.len = 0;

    // Set to non-connectable and non-scannable advertising
    //adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
}

/*
      m_adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
      err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
      APP_ERROR_CHECK(err_code);
*/  
    

    // Configure advertising parameters
    ble_gap_adv_params_t adv_params; 
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    if(Son == 1)
    {
      adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    }
    else
    {
      adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    }
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    if(Son == 1)
    {
      adv_params.interval        = APP_ADV_INTERVAL;
    }
    else
    {
      adv_params.interval        = APP_ADV_INTERVAL_LONG;
    }
    
    if(Son == 1)
    {
      // Use full m_adv_data with scan response
        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    }
    else
    {
          // Create temporary struct with scan_rsp_data disabled
        ble_gap_adv_data_t adv_data_no_scan_rsp;
        adv_data_no_scan_rsp.adv_data = m_adv_data.adv_data;
        adv_data_no_scan_rsp.scan_rsp_data.p_data = NULL;
        adv_data_no_scan_rsp.scan_rsp_data.len = 0;
    

        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &adv_data_no_scan_rsp, &adv_params);
    }
    AppErrorCheck(err_code);

    // Start advertising
    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    AppErrorCheck(err_code);
    m_advertising_running = true;
    
}

void GetDataUUID(uint8_t *new_manuf_data)
{
    ble_gap_addr_t new_addr;
    ret_code_t err_code;
    new_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
        uint8_t* p = (uint8_t*)MemReadAdr;
      for(uint8_t i= 0;i<6;i++)
  {
    new_addr.addr[5-i] = *p;
    p++;
  }
  if((new_addr.addr[0] == 0xff) && (new_addr.addr[1] == 0xff) && (new_addr.addr[2] == 0xff) && (new_addr.addr[3] == 0xff) && (new_addr.addr[4] == 0xff) && (new_addr.addr[5] == 0xff))
  {
      ble_gap_addr_t ble_addr;
      flash_empty = 1;
  //memcpy(ble_addr.addr,&unique_id,6);
      // Get BLE address.
    err_code = sd_ble_gap_addr_get(&ble_addr);
    APP_ERROR_CHECK(err_code);
    new_addr.addr[5] = ble_addr.addr[5];
    new_addr.addr[4] = ble_addr.addr[4];
    new_addr.addr[3] = ble_addr.addr[3];
    new_addr.addr[2] = ble_addr.addr[2];
    new_addr.addr[1] = ble_addr.addr[1];
    new_addr.addr[0] = ble_addr.addr[0];
  }
  else
  {
    flash_empty = 0;
  }
   err_code = sd_ble_gap_addr_set(&new_addr);
   APP_ERROR_CHECK(err_code);

  new_manuf_data[0] = 0x12;
  new_manuf_data[1] = 0x19;
//  new_manuf_data[2] = 0x24;
  new_manuf_data[2] = BattPwr;
  for(uint8_t i= 3;i<APP_BEACON_INFO_LENGTH1;i++)
  {
    new_manuf_data[i] = *(p+2);
    p++;
  }


}

void    GetName(uint8_t * pnew_name)
{
     ret_code_t     err_code;
  char addr_str[13];
  uint8_t max_len = sizeof(addr_str);
//  uint32_t id0 = NRF_FICR->DEVICEID[0];
// uint32_t id1 = NRF_FICR->DEVICEID[1];

uint32_t id0 = NRF_FICR->DEVICEADDR[0];
uint32_t id1 = NRF_FICR->DEVICEADDR[1];

// Optional: Combine into 64-bit ID
//uint64_t unique_id = ((uint64_t)id1 << 32) | id0;
  ble_gap_addr_t ble_addr;
  //memcpy(ble_addr.addr,&unique_id,6);
      // Get BLE address.
    err_code = sd_ble_gap_addr_get(&ble_addr);
    APP_ERROR_CHECK(err_code);
//    ret_code_t err_code = sd_ble_gap_addr_get(&ble_addr);
//    APP_ERROR_CHECK(err_code);
    // BLE addresses are stored LSB first, so print in reverse for readability
    snprintf(addr_str, max_len,
             "%02X%02X%02X%02X%02X%02X",
             ble_addr.addr[5],
             ble_addr.addr[4],
             ble_addr.addr[3],
             ble_addr.addr[2],
             ble_addr.addr[1],
             ble_addr.addr[0]);
    
              pnew_name[0] = 'W';
              pnew_name[1] = 'L';
              pnew_name[2] = 'I';
              pnew_name[3] = 'n';
              pnew_name[4] = 'R';
              pnew_name[5] = 'F';
              pnew_name[6] = 'z';
              pnew_name[7] = ':';
              memcpy(pnew_name+8,addr_str,12);
}

#if 0
static void set_custom_ble_address(void)
{
    ble_gap_addr_t new_addr;
    ret_code_t err_code;

    new_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    new_addr.addr[0] = 0x2b;
    new_addr.addr[1] = 0xf4;
    new_addr.addr[2] = 0x7e;
    new_addr.addr[3] = 0xFE;
    new_addr.addr[4] = 0xd3;
    new_addr.addr[5] = 0xC0 | (0xf9 & 0x3F);  // forces top two bits to 11

    err_code = sd_ble_gap_addr_set(&new_addr);
    APP_ERROR_CHECK(err_code);
}
#endif