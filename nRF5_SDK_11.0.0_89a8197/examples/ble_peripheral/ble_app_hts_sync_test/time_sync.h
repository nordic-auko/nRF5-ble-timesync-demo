#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

/**@brief Data handler type. */
typedef void (*ts_evt_handler_t)(uint32_t time);

typedef struct
{
//    ts_evt_handler_t evt_handler;     /** Event handler */
    uint8_t          rf_chn;          /** RF Channel [0-80] */
    uint8_t          rf_addr[5];      /** 5-byte RF address */
    uint8_t          ppi_chns[3];     /** PPI channels */
    uint8_t          ppi_chhg;        /** PPI Channel Group */
    NRF_TIMER_Type * high_freq_timer[2]; /** 16 MHz timer (e.g. NRF_TIMER2) */
    NRF_RTC_Type   * rtc;
} ts_params_t;

/**@brief SoftDevice system event handler. Must be called when a system event occurs */
void ts_on_sys_evt(uint32_t sys_evt);


uint32_t ts_init(const ts_params_t * p_params);

uint32_t ts_enable(void);
uint32_t ts_disable(void);

uint32_t ts_tx_start(uint32_t sync_freq_hz);

uint32_t ts_tx_stop(void);


#endif /* __TIME_SYNC_H__ */
