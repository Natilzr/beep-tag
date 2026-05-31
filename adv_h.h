#ifndef ADV_H_H
#define ADV_H_H

#include "nrf_fstorage.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_gap.h"
#include "app_timer.h"

#define APP_BEACON_INFO_LENGTH1    0x1B         //27
//#define APP_ADV_INTERVAL                3200   //every 2 sec
#define APP_ADV_INTERVAL                4800   //every 2 sec
//#define APP_ADV_INTERVAL_LONG           16000   //every 10 sec
#define APP_ADV_INTERVAL_LONG           9600   //every 6 sec
//#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */
#define APP_ADV_DATA_LENGTH1             0x19   //25
#define APP_DEVICE_TYPE1 0xff 
#define APP_COMPANY_IDENTIFIER1  0x004c // (apple)
#define APP_BEACON_UUID1                0x12,0x19,0x24,0xaa,\
                                        0x7f,0x38,0x53,0xfd,\
                                        0x99,0x71,0xe6,0x69,\
                                        0xb6,0x33,0x65,0xcc,\
                                        0x37,0x9f,0xaa,0x4f,\
                                        0x3f,0x97,0x84,0xb4,\
                                        0x59,0x02,0x00             /**< Proprietary UUID for Beacon. 23*/
                                          
                                          
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */                                          
#define DYNAMIC_NAME_LEN         20//19
                                          
void advertising_update_runtime(uint8_t *new_manuf_data, uint8_t Son);
void GetDataUUID(uint8_t *new_manuf_data);
void GetName(uint8_t * pnew_name);
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
static bool m_advertising_running = false;

extern NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage);
extern uint8_t NumRec;
extern uint8_t new_name[20];
extern uint8_t new_manuf_data[APP_BEACON_INFO_LENGTH1];
extern uint32_t MemAdr;
extern uint32_t MemReadAdr;

extern ble_gap_adv_data_t m_adv_data;
extern uint8_t m_adv_handle;

extern void start_inactiv_T(void);


#endif