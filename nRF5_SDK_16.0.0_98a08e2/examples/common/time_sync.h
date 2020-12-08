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

#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrfx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TS_SOC_OBSERVER_PRIO 0

#define TIME_SYNC_TIMER_MAX_VAL (40000)	// sync timer time base: 2.5 ms
#define TIME_SYNC_RTC_MAX_VAL   (0xFFFFFF)

typedef struct
{
    NRF_TIMER_Type * high_freq_timer[2]; /** 16 MHz timer (e.g. NRF_TIMER2). NOTE: debug toggling only available if TIMER3 or TIMER4 is used for high_freq_timer[0]*/
    NRF_EGU_Type   * egu;
    IRQn_Type        egu_irq_type;
} ts_init_t;


/**@brief Initialize time sync library
 * 
 * @param[in] p_params Parameters
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_init(const ts_init_t * p_init);

typedef struct
{
	uint32_t startTick;
	uint32_t targetTick;
	uint32_t lastSync;
	uint16_t syncPacketCnt;
	uint16_t usedPacketCnt;
} ts_trigger_evt_t;

typedef void (*ts_evt_handler_t)(const ts_trigger_evt_t* evt);

uint32_t ts_set_trigger(uint32_t target_tick, uint32_t ppi_endpoint, ts_evt_handler_t callback);

typedef struct
{
	uint8_t rf_chn; 	/** RF Channel [0-80] */
	uint8_t rf_addr[5]; /** 5-byte RF address */
} ts_rf_config_t;

/**@brief Enable time sync library. This will enable reception of sync packets.
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_enable(const ts_rf_config_t* p_rf_config);

/**@brief Disable time sync library. 
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_disable(void);

/**@brief Start sync packet transmission (become timing master).
 *
 * @note @ref ts_enable() must be called prior to calling this function
 * @note Expect some jitter depending on BLE activity.
 *
 * @param[in] sync_freq_hz Frequency of transmitted sync packets. 
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_tx_start(uint32_t sync_freq_hz);

/**@brief Stop sync packet transmission (become timing slave again).
 *
 * @retval NRF_SUCCESS if successful 
 */
uint32_t ts_tx_stop(void);

/**@brief Get timestamp value in 16 MHz ticks
 *
 * @note 32-bit variable overflows after ~268 seconds
 * 
 * @param[in] ppi_chn PPI channel to use for timer capture. Channel is not used after function exits.
 * 
 * @retval timestamp value [1 second/16 MHz]
 */
uint32_t ts_timestamp_get_ticks_u32(void);

/**@brief Get timestamp value in 16 MHz ticks
 *
 * @note Internal 32-bit counter overflows after (2^32 * @ref TIME_SYNC_TIMER_MAX_VAL) / 16 000 000 seconds. 
 * 
 * @param[in] ppi_chn PPI channel to use for timer capture. Channel is not used after function exits.
 * 
 * @retval timestamp value [1 second/16 MHz]
 */
uint64_t ts_timestamp_get_ticks_u64(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIME_SYNC_H__ */
