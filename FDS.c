///FDS.c


#include "fds.h"

#define STATUS_FILE_ID      0x1001
#define STATUS_RECORD_KEY   0x2001

typedef struct
{
    bool    stay_awake;
    uint8_t reset_reason;   // battery / wdt / ble_cmd / normal
} device_status_t;

// Reset reasons
#define RESET_REASON_NORMAL     0x00
#define RESET_REASON_BATTERY    0x01
#define RESET_REASON_WDT        0x02
#define RESET_REASON_BLE_CMD    0x03

// ============================================================
// NVM Read / Write
// ============================================================

void nvm_write_status(device_status_t *status)
{
    fds_record_t        record;
    fds_record_desc_t   desc = {0};
    fds_find_token_t    tok  = {0};

    record.file_id          = STATUS_FILE_ID;
    record.key              = STATUS_RECORD_KEY;
    record.data.p_data      = status;
    record.data.length_words = CEIL_DIV(sizeof(device_status_t), 4);

    if (fds_record_find(STATUS_FILE_ID, STATUS_RECORD_KEY, &desc, &tok) == FDS_SUCCESS)
    {
        // Record exists - update it
        fds_record_update(&desc, &record);
    }
    else
    {
        // First time - write new record
        fds_record_write(&desc, &record);
    }

    // Wait for write to complete
    fds_gc();
}

bool nvm_read_status(device_status_t *status)
{
    fds_record_desc_t   desc = {0};
    fds_find_token_t    tok  = {0};
    fds_flash_record_t  flash_record;

    if (fds_record_find(STATUS_FILE_ID, STATUS_RECORD_KEY, &desc, &tok) == FDS_SUCCESS)
    {
        fds_record_open(&desc, &flash_record);
        memcpy(status, flash_record.p_data, sizeof(device_status_t));
        fds_record_close(&desc);
        return true;
    }

    return false;   // No record found (first boot)
}

// ============================================================
// Force Sleep - saves state before sleeping
// ============================================================

void force_sleep(uint8_t reason)
{
    device_status_t status;

    stay_awake        = false;
    status.stay_awake = false;
    status.reset_reason = reason;

    nvm_write_status(&status);      // Persist before sleeping

    ble_disconnect();
    peripherals_shutdown();
    nrf_drv_wdt_feed();

    sd_app_evt_wait();
}

void go_to_sleep_command(void)
{
    force_sleep(RESET_REASON_BLE_CMD);
}