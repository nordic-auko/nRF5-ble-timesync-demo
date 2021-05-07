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

#include "time_sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "app_util_platform.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "nrf_atomic.h"
#include "nrf_balloc.h"
#include "nrf_error.h"
#include "nrfx_ppi.h"
#include "nrf_soc.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdm.h"

#define NRF_LOG_MODULE_NAME time_sync
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#if   defined ( __CC_ARM )
#define TX_CHAIN_DELAY_PRESCALER_0 (699 - 235)
#elif defined ( __ICCARM__ )
#define TX_CHAIN_DELAY_PRESCALER_0 (703 - 235)
#elif defined ( __GNUC__ )
#define TX_CHAIN_DELAY_PRESCALER_0 (704 - 237)
#endif

#define SYNC_TIMER_PRESCALER 0
#define SYNC_RTC_PRESCALER 0

#if SYNC_TIMER_PRESCALER == 0
#define TX_CHAIN_DELAY TX_CHAIN_DELAY_PRESCALER_0
#else
#error Invalid prescaler value
#endif

#if TIME_SYNC_TIMER_MAX_VAL < 10000
#error Timer values below 10000 not supported
#endif

static void ts_on_sys_evt(uint32_t sys_evt, void * p_context);

NRF_SDH_SOC_OBSERVER(timesync_soc_obs,     \
                     TS_SOC_OBSERVER_PRIO, \
                     ts_on_sys_evt, 0);

#define TS_LEN_US                            (1000UL)
#define TX_LEN_EXTENSION_US                  (1000UL)
#define TS_SAFETY_MARGIN_US                  (500UL)   /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US                  (700UL)   /**< The timeslot activity should request an extension this long before end of timeslot. */


#define MAIN_DEBUG                           0x12345678UL

static void ppi_sync_timer_adjust_configure(bool do_count);
static void ppi_sync_timer_adjust_enable(void);
static void ppi_radio_rx_disable(void);
static void ppi_radio_rx_configure(void);
static void ppi_radio_tx_configure(void);
static uint32_t ppi_sync_trigger_configure(uint32_t ppi_endpoint);

typedef struct
{
    uint8_t                 rf_chn;          /** RF Channel [0-80] */
    uint8_t                 rf_addr[5];      /** 5-byte RF address */
    nrf_ppi_channel_t       ppi_chns[5];     /** PPI channels */
    nrf_ppi_channel_group_t ppi_chg;        /** PPI Channel Group */
    NRF_TIMER_Type *        high_freq_timer[2]; /** 16 MHz timer (e.g. NRF_TIMER2). NOTE: debug toggling only available if TIMER3 or TIMER4 is used for high_freq_timer[0]*/
    NRF_EGU_Type   *        egu;
    IRQn_Type               egu_irq_type;
} ts_params_t;

typedef PACKED_STRUCT
{
    int32_t  timer_val;
    int32_t  rtc_val;
    uint32_t counter_val;
} sync_pkt_t;

static uint8_t          m_sync_pkt_ringbuf[10][sizeof(sync_pkt_t)];
static nrf_atomic_u32_t m_sync_pkt_ringbuf_idx;

static nrf_atomic_flag_t m_timeslot_session_open;
static nrf_atomic_flag_t m_pending_close;
static nrf_atomic_u32_t  m_blocked_cancelled_count;
static uint32_t          m_total_timeslot_length = 0;
static uint32_t          m_timeslot_distance = 0;
static uint32_t          m_tx_slot_retry_count = 0;
static uint32_t          m_usec_since_sync_packet = 0;
static uint32_t          m_usec_since_tx_offset_calc = 0;
static ts_params_t       m_params;

static nrf_atomic_flag_t m_send_sync_pkt = false;
static nrf_atomic_flag_t m_timer_update_in_progress = false;

static bool m_synchronized = false;

static volatile int64_t  m_master_counter_diff = 0;
static nrf_atomic_u32_t  m_rcv_count      = 0;

static nrf_atomic_u32_t  mp_curr_adj_pkt;
static nrf_atomic_u32_t  m_curr_adj_timer;
static nrf_atomic_u32_t  m_curr_adj_counter;

static ts_evt_handler_t m_callback;
static nrf_atomic_u32_t m_tick_target;
static nrf_atomic_u32_t m_sync_packet_count = 0;
static nrf_atomic_u32_t m_used_packet_count = 0;
static nrf_atomic_u32_t m_last_sync = 0;

static nrf_ppi_channel_t m_timestamp_trigger_ppi[2];
static bool m_timestamp_trigger_set = false;

static volatile enum
{
    RADIO_STATE_IDLE, /* Default state */
    RADIO_STATE_RX,   /* Waiting for packets */
    RADIO_STATE_TX    /* Trying to transmit packet */
} m_radio_state = RADIO_STATE_IDLE;

static void ppi_counter_timer_triggered_capture_configure(nrf_ppi_channel_t chn[2], uint32_t eep);
static bool sync_timer_offset_compensate(sync_pkt_t * p_pkt);
static void timeslot_begin_handler(void);
static void timeslot_end_handler(void);

/**< This will be used when requesting the first timeslot or any time a timeslot is blocked or cancelled. */
static nrf_radio_request_t m_timeslot_req_earliest = {
        NRF_RADIO_REQ_TYPE_EARLIEST,
        .params.earliest = {
            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
            NRF_RADIO_PRIORITY_NORMAL,
            TS_LEN_US,
            100000 /* Expect timeslot within 100 ms */
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_request_t m_timeslot_req_normal = {
        NRF_RADIO_REQ_TYPE_NORMAL,
        .params.normal = {
            NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
            NRF_RADIO_PRIORITY_NORMAL,
            0,
            TS_LEN_US
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_normal = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
        .params.request = {
                (nrf_radio_request_t*) &m_timeslot_req_normal
        }};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_sched_next_earliest = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END,
        .params.request = {
                (nrf_radio_request_t*) &m_timeslot_req_earliest
        }};

/**< This will be used at the end of each timeslot to request an extension of the timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_extend = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND,
        .params.extend = {TX_LEN_EXTENSION_US}
        };

/**< This will be used at the end of each timeslot to request the next timeslot. */
static nrf_radio_signal_callback_return_param_t m_rsc_return_no_action = {
        NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE,
        .params.request = {NULL}
        };

static void increment_desync_timeout(uint32_t increment_usec)
{
    if (m_send_sync_pkt)
    {
        // No desync as transmitter
        return;
    }

    m_usec_since_sync_packet += increment_usec;
    if (m_usec_since_sync_packet >= TIME_SYNC_DESYNC_TIMEOUT)
    {
        m_params.egu->TASKS_TRIGGER[3] = 1;
        m_usec_since_sync_packet = 0; // Reset to suppress needless irq generation
    }
}

static sync_pkt_t * tx_buf_get(void)
{
    uint32_t idx;

    idx = nrf_atomic_u32_fetch_add(&m_sync_pkt_ringbuf_idx, 1) % ARRAY_SIZE(m_sync_pkt_ringbuf);

    return (sync_pkt_t *) m_sync_pkt_ringbuf[idx];
}

volatile uint32_t m_prev_sync_pkt_timer;
volatile uint32_t m_prev_sync_pkt_counter;

void RADIO_IRQHandler(void)
{
    if (NRF_RADIO->EVENTS_END != 0)
    {
        NRF_RADIO->EVENTS_END = 0;
        (void)NRF_RADIO->EVENTS_END;

        if (m_radio_state == RADIO_STATE_RX &&
           (NRF_RADIO->CRCSTATUS & RADIO_CRCSTATUS_CRCSTATUS_Msk) == (RADIO_CRCSTATUS_CRCSTATUS_CRCOk << RADIO_CRCSTATUS_CRCSTATUS_Pos))
        {
            sync_pkt_t * p_pkt;
            bool         adjustment_procedure_started;

            p_pkt = (sync_pkt_t *) NRF_RADIO->PACKETPTR;

            if (p_pkt->timer_val <= 2)
            {
                // Ignore packet due to potential missed counter increment
                // TODO: See if this can be tightened
                goto resume_radio_rx;
            }


            adjustment_procedure_started = sync_timer_offset_compensate(p_pkt);
            // m_trigger_evt.params.triggered.sync_packet_count++; // rx packet received
            nrf_atomic_u32_add(&m_sync_packet_count, 1);

            if (adjustment_procedure_started)
            {
                m_prev_sync_pkt_timer = p_pkt->timer_val;
                m_prev_sync_pkt_counter = p_pkt->counter_val;
                p_pkt = tx_buf_get();

                NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;

                m_usec_since_sync_packet = 0;
            }

            nrf_atomic_u32_add(&m_rcv_count, 1);
        }

resume_radio_rx:

        NRF_RADIO->TASKS_START = 1;
    }
}

/**@brief   Function for handling timeslot events.
 */
static nrf_radio_signal_callback_return_param_t * radio_callback (uint8_t signal_type)
{
    // NOTE: This callback runs at lower-stack priority (the highest priority possible).
    switch (signal_type) {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        // TIMER0 is pre-configured for 1Mhz.
        NRF_TIMER0->TASKS_STOP          = 1;
        NRF_TIMER0->TASKS_CLEAR         = 1;
        NRF_TIMER0->MODE                = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;

        if (m_send_sync_pkt)
        {
            NRF_TIMER0->INTENSET  = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos);
        }
        else
        {
            NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos) |
                                   (TIMER_INTENSET_COMPARE1_Set << TIMER_INTENSET_COMPARE1_Pos);
        }
        NRF_TIMER0->CC[0]               = (TS_LEN_US - TS_SAFETY_MARGIN_US);
        NRF_TIMER0->CC[1]               = (TS_LEN_US - TS_EXTEND_MARGIN_US);
        NRF_TIMER0->BITMODE             = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
        NRF_TIMER0->TASKS_START         = 1;


        NRF_RADIO->POWER                = (RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos);

        NVIC_EnableIRQ(TIMER0_IRQn);

        m_total_timeslot_length = 0;
        m_tx_slot_retry_count = 0;

        timeslot_begin_handler();

        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
        if (NRF_TIMER0->EVENTS_COMPARE[0] &&
           (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENCLR_COMPARE0_Pos)))
        {
            NRF_TIMER0->TASKS_STOP  = 1;
            NRF_TIMER0->EVENTS_COMPARE[0] = 0;
            (void)NRF_TIMER0->EVENTS_COMPARE[0];

            // This is the "timeslot is about to end" timeout

            timeslot_end_handler();

            // Schedule next timeslot
            if (m_send_sync_pkt)
            {
                m_timeslot_req_normal.params.normal.distance_us = m_timeslot_distance/* - m_tx_offset*/;

                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_normal;
            }
            else
            {
                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_earliest;
            }
        }

        if (NRF_TIMER0->EVENTS_COMPARE[1] &&
           (NRF_TIMER0->INTENSET & (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENCLR_COMPARE1_Pos)))
        {
            NRF_TIMER0->EVENTS_COMPARE[1] = 0;
            (void)NRF_TIMER0->EVENTS_COMPARE[1];

            // This is the "try to extend timeslot" timeout

            if (m_total_timeslot_length < (128000000UL - 5000UL - TX_LEN_EXTENSION_US) && !m_send_sync_pkt)
            {
                // Request timeslot extension if total length does not exceed 128 seconds
                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_extend;
            }
            else if (!m_send_sync_pkt)
            {
                return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_earliest;
            }
        }
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
        RADIO_IRQHandler();
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
        timeslot_end_handler();

        increment_desync_timeout(TX_LEN_EXTENSION_US);

        return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_sched_next_earliest;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
        // Extension succeeded: update timer
        NRF_TIMER0->TASKS_STOP          = 1;
        NRF_TIMER0->EVENTS_COMPARE[0]   = 0;
        NRF_TIMER0->EVENTS_COMPARE[1]   = 0;
        NRF_TIMER0->CC[0]               += (TX_LEN_EXTENSION_US - 25);
        NRF_TIMER0->CC[1]               += (TX_LEN_EXTENSION_US - 25);
        NRF_TIMER0->TASKS_START         = 1;

        // Keep track of total length
        m_total_timeslot_length += TX_LEN_EXTENSION_US;

        increment_desync_timeout(TX_LEN_EXTENSION_US);
        break;

    default:
        app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
        break;
    };

    // Fall-through return: return with no action request
    return (nrf_radio_signal_callback_return_param_t*) &m_rsc_return_no_action;
}

static void update_radio_parameters(sync_pkt_t * p_pkt)
{
    // TX power
    NRF_RADIO->TXPOWER  = RADIO_TXPOWER_TXPOWER_0dBm   << RADIO_TXPOWER_TXPOWER_Pos;

    // RF bitrate
    NRF_RADIO->MODE     = RADIO_MODE_MODE_Nrf_2Mbit       << RADIO_MODE_MODE_Pos;

    // Fast startup mode
    NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;

    // CRC configuration
    NRF_RADIO->CRCCNF  = RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos;
    NRF_RADIO->CRCINIT = 0xFFFFFFUL;      // Initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

    // Packet format
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled     << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big           << RADIO_PCNF1_ENDIAN_Pos)  |
                       (4                                << RADIO_PCNF1_BALEN_Pos)   |
                       (sizeof(sync_pkt_t)               << RADIO_PCNF1_STATLEN_Pos) |
                       (sizeof(sync_pkt_t)               << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->PACKETPTR = (uint32_t) p_pkt;

    // Radio address config
    NRF_RADIO->PREFIX0 = m_params.rf_addr[0];
    NRF_RADIO->BASE0   = (m_params.rf_addr[1] << 24 | m_params.rf_addr[2] << 16 | m_params.rf_addr[3] << 8 | m_params.rf_addr[4]);

    NRF_RADIO->TXADDRESS   = 0; // use logical address 0 for transmit
    NRF_RADIO->RXADDRESSES = (1 << 0); // enable logical address 0 for receive

    NRF_RADIO->FREQUENCY = m_params.rf_chn;
    NRF_RADIO->TXPOWER   = RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;

    NRF_RADIO->EVENTS_END = 0;

    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

    NVIC_EnableIRQ(RADIO_IRQn);
}

void timeslot_end_handler(void)
{
    NRF_RADIO->TASKS_DISABLE = 1;
    NRF_RADIO->INTENCLR      = 0xFFFFFFFF;

    ppi_radio_rx_disable();

    m_radio_state           = RADIO_STATE_IDLE;
}

void timeslot_begin_handler(void)
{
    sync_pkt_t * p_pkt;

    m_total_timeslot_length = 0;

    if (!m_send_sync_pkt)
    {
        if (m_radio_state    != RADIO_STATE_RX ||
            NRF_RADIO->STATE != (RADIO_STATE_STATE_Rx << RADIO_STATE_STATE_Pos))
        {
            p_pkt = tx_buf_get();

            update_radio_parameters(p_pkt);

            NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
            NRF_RADIO->TASKS_RXEN = 1;

            ppi_radio_rx_configure();

            m_radio_state = RADIO_STATE_RX;
        }

        return;
    }

    if (m_radio_state == RADIO_STATE_RX)
    {
        // Packet transmission has now started (state change from RX).
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0)
        {
            __NOP();
        }
    }

    p_pkt = tx_buf_get();

    ppi_radio_tx_configure();
    update_radio_parameters(p_pkt);

    NRF_RADIO->SHORTS     = RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->TASKS_TXEN = 1;

    while (NRF_RADIO->EVENTS_READY == 0)
    {
        // PPI is used to trigger sync timer capture when radio is ready
        // Radio will automatically start transmitting once ready, so the captured timer value must be copied into radio packet buffer ASAP
        __NOP();
    }

    p_pkt->timer_val   = m_params.high_freq_timer[0]->CC[1];
    p_pkt->counter_val = m_params.high_freq_timer[1]->CC[1];

//    p_pkt->rtc_val     = m_params.rtc->COUNTER;

    // m_trigger_evt.params.triggered.sync_packet_count++; // tx packet sent
    nrf_atomic_u32_add(&m_sync_packet_count, 1);
    m_radio_state = RADIO_STATE_TX;
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
void ts_on_sys_evt(uint32_t sys_evt, void * p_context)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            break;
        case NRF_EVT_RADIO_BLOCKED:
        case NRF_EVT_RADIO_CANCELED:
        {
            if (!m_pending_close)
            {
                /*
                 * This is caused by a conflict with an active BLE session.
                 * This will mess up the tx frequency, because the next request is scheduled immediately
                 */
                // Blocked events are rescheduled with normal priority. They could also
                // be rescheduled with high priority if necessary.
                if (m_send_sync_pkt && m_tx_slot_retry_count < 5)
                {
                    ++m_tx_slot_retry_count;
                    m_timeslot_req_normal.params.normal.distance_us = m_timeslot_distance * (m_tx_slot_retry_count + 2);
                    uint32_t err_code = sd_radio_request((nrf_radio_request_t*) &m_timeslot_req_normal);
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    uint32_t err_code = sd_radio_request((nrf_radio_request_t*) &m_timeslot_req_earliest);
                    APP_ERROR_CHECK(err_code);
                }

                nrf_atomic_u32_add(&m_blocked_cancelled_count, 1);
            }
            break;
        }
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            NRF_LOG_ERROR("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN\r\n");
            app_error_handler(MAIN_DEBUG, __LINE__, (const uint8_t*)__FILE__);
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            nrf_atomic_flag_clear(&m_timeslot_session_open);
            nrf_atomic_flag_clear(&m_pending_close);
            NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_CLOSED\r\n");
            break;
        case NRF_EVT_RADIO_SESSION_IDLE:
        {
            NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_IDLE\r\n");

            uint32_t err_code = sd_radio_session_close();
            APP_ERROR_CHECK(err_code);
            break;
        }
        default:
            // No implementation needed.
            NRF_LOG_INFO("Event: 0x%08x\r\n", sys_evt);
            break;
    }
}

static void timestamp_counter_start(void)
{
    // m_params.high_freq_timer[1] (NRF_TIMER) is used in counter mode to count the number of sync timer overflows/resets (m_params.high_freq_timer[0])
    // When timestamp API is used, the number of overflows/resets + current value of sync timer must be added up to give accurate timestamp information
    m_params.high_freq_timer[1]->TASKS_STOP  = 1;
    m_params.high_freq_timer[1]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[1]->PRESCALER   = 0;
    m_params.high_freq_timer[1]->MODE        = TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos;;
    m_params.high_freq_timer[1]->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    m_params.high_freq_timer[1]->TASKS_START = 1;
}

static void sync_timer_start(void)
{
    // m_params.high_freq_timer[0] (NRF_TIMER) is the always-running sync timer
    // The timing master never adjusts this timer
    // The timing slave(s) adjusts this timer whenever a sync packet is received and the logic determines that there is
    m_params.high_freq_timer[0]->TASKS_STOP  = 1;
    m_params.high_freq_timer[0]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[0]->PRESCALER   = SYNC_TIMER_PRESCALER;
    m_params.high_freq_timer[0]->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    m_params.high_freq_timer[0]->CC[0]       = TIME_SYNC_TIMER_MAX_VAL;
    m_params.high_freq_timer[0]->CC[1]       = 0xFFFFFFFF;
    m_params.high_freq_timer[0]->CC[2]       = 0xFFFFFFFF;
    m_params.high_freq_timer[0]->CC[3]       = 0xFFFFFFFF;

    // if (m_params.high_freq_timer[0] == NRF_TIMER3 || m_params.high_freq_timer[0] == NRF_TIMER4)
    // {
    //     // TIMERS 0,1, and 2 only have 4 compare registers
    //     m_params.high_freq_timer[0]->CC[5]   = TIME_SYNC_TIMER_MAX_VAL / 2; // Only used for debugging purposes such as pin toggling
    // }

    m_params.high_freq_timer[0]->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    m_params.high_freq_timer[0]->TASKS_START = 1;
}

uint32_t ts_set_trigger(uint32_t target_tick, uint32_t ppi_endpoint)
{
    if (!m_timeslot_session_open)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (ppi_sync_trigger_configure(ppi_endpoint) != NRF_SUCCESS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    nrf_atomic_u32_store(&m_sync_packet_count, 0);
    nrf_atomic_u32_store(&m_used_packet_count, 0);

    // TODO: is there a way to check if the target value is plausible?
    // m_trigger_evt.params.triggered.tick_start = ts_timestamp_get_ticks_u64() / TIME_SYNC_TIMER_MAX_VAL;

    m_params.high_freq_timer[1]->CC[4] = target_tick - m_master_counter_diff;
    nrf_atomic_u32_store(&m_tick_target, target_tick);
    nrf_ppi_channel_enable(m_params.ppi_chns[4]); // activate trigger
    return NRF_SUCCESS;
}

uint32_t ts_set_timestamp_trigger(uint32_t ppi_event_endpoint)
{
    // TODO: Check if other timers can be used also
    if (m_params.high_freq_timer[0] != NRF_TIMER3 && m_params.high_freq_timer[0] != NRF_TIMER4)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (m_params.high_freq_timer[1] != NRF_TIMER3 && m_params.high_freq_timer[1] != NRF_TIMER4)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (!m_timestamp_trigger_set)
    {
        nrfx_err_t err;

        for (int i = 0; i < ARRAY_SIZE(m_timestamp_trigger_ppi); ++i)
        {
            err = nrfx_ppi_channel_alloc(&m_timestamp_trigger_ppi[i]);
            if (err != NRFX_SUCCESS)
            {
                return err;
            }
        }
    }

    ppi_counter_timer_triggered_capture_configure(m_timestamp_trigger_ppi, ppi_event_endpoint);

    m_timestamp_trigger_set = true;

    return NRF_SUCCESS;
}

void SWI3_EGU3_IRQHandler(void)
{
    if (NRF_EGU3->EVENTS_TRIGGERED[0] != 0)
    {
        m_master_counter_diff = ((sync_pkt_t *) mp_curr_adj_pkt)->counter_val - m_curr_adj_counter;
        nrf_atomic_u32_add(&m_used_packet_count, 1);
        nrf_atomic_u32_store(&m_last_sync, (m_curr_adj_counter - m_master_counter_diff));

        m_params.high_freq_timer[0]->CC[2] = 0;

        nrf_atomic_flag_clear(&m_timer_update_in_progress);

        NRF_EGU3->EVENTS_TRIGGERED[0] = 0;

        if (!m_synchronized)
        {
            m_synchronized = true;

            if (m_callback)
            {
                ts_evt_t evt =
                {
                    .type = TS_EVT_SYNCHRONIZED,
                };

                m_callback(&evt);
            }
        }
    }

    if (NRF_EGU3->EVENTS_TRIGGERED[2] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[2] = 0;
        nrf_ppi_channel_disable(m_params.ppi_chns[4]);

        if (m_callback)
        {
            ts_evt_t trigger_evt =
            {
                .type = TS_EVT_TRIGGERED,
                .params.triggered.tick_target = m_tick_target,
                .params.triggered.last_sync = m_last_sync,
                .params.triggered.sync_packet_count = m_sync_packet_count,
                .params.triggered.used_packet_count = m_used_packet_count,
            };
            m_callback(&trigger_evt);
        }
    }

    if (NRF_EGU3->EVENTS_TRIGGERED[3] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[3] = 0;

        if (m_synchronized)
        {
            m_synchronized = false;

            if (m_callback)
            {
                ts_evt_t evt =
                {
                    .type = TS_EVT_DESYNCHRONIZED,
                };

                m_callback(&evt);
            }
        }
    }

    if (NRF_EGU3->EVENTS_TRIGGERED[4] != 0)
    {
        // TODO: Remove this event, as it is not currently used
        NRF_EGU3->EVENTS_TRIGGERED[4] = 0;
    }

    if (NRF_EGU3->EVENTS_TRIGGERED[5] != 0)
    {
        uint64_t timestamp;
        uint32_t counter_val;

        NRF_EGU3->EVENTS_TRIGGERED[5] = 0;

        counter_val = m_params.high_freq_timer[1]->CC[5];

        if ((m_params.high_freq_timer[0]->CC[5] <= 5) || (m_params.high_freq_timer[0]->CC[5] == TIME_SYNC_TIMER_MAX_VAL) || (m_params.high_freq_timer[0]->CC[2] != 0 && m_params.high_freq_timer[0]->CC[5] == m_params.high_freq_timer[0]->CC[2]))
        {
            // Check if counter increment was caught
            m_params.high_freq_timer[1]->TASKS_CAPTURE[5] = 1;
            if (m_params.high_freq_timer[1]->CC[5] != counter_val)
            {
                counter_val = m_params.high_freq_timer[1]->CC[5];
            }
        }

        timestamp  = counter_val;
        timestamp += m_master_counter_diff;
        timestamp *= TIME_SYNC_TIMER_MAX_VAL;
        timestamp += m_params.high_freq_timer[0]->CC[5];

        if (m_callback)
        {
            ts_evt_t evt =
            {
                .type = TS_EVT_TIMESTAMP,
                .params.timestamp = timestamp,
            };

            m_callback(&evt);
        }
    }
}

static inline bool sync_timer_offset_compensate(sync_pkt_t * p_pkt)
{
    uint32_t peer_timer;
    uint32_t local_timer;
    int32_t timer_offset;
    bool wrapped = false;

    if (m_timer_update_in_progress)
    {
        return false;
    }

    peer_timer  = p_pkt->timer_val;
    peer_timer += TX_CHAIN_DELAY;
    if (peer_timer > TIME_SYNC_TIMER_MAX_VAL)
    {
        peer_timer -= TIME_SYNC_TIMER_MAX_VAL;
        p_pkt->counter_val += 1;
        wrapped = true;
    }

    local_timer = m_params.high_freq_timer[0]->CC[1];
    timer_offset = local_timer - peer_timer;

    // NRF_LOG_INFO("timer_offset: %d (wrapped: %d)", timer_offset, wrapped);
    // NRF_LOG_INFO("Local: %lu, Remote: %lu", local_timer, peer_timer);

    if (timer_offset == 0 || timer_offset == TIME_SYNC_TIMER_MAX_VAL)
    {
        // Already in sync
        nrf_atomic_u32_add(&m_used_packet_count, 1);
        // nrf_atomic_u32_store(&m_last_sync, p_pkt->counter_val);
        return false;
    }

    if (wrapped && (local_timer >= (TIME_SYNC_TIMER_MAX_VAL - 50)))
    {
        // Too close
        // Todo: see if local counter increment can be accounted for
        // NRF_LOG_INFO("SKIPPED");
        return false;
    }

    if (timer_offset < 0)
    {
        // Local timer is ahead of peer: cut current cycle short
        m_params.high_freq_timer[0]->CC[2] = TIME_SYNC_TIMER_MAX_VAL + timer_offset;
        ppi_sync_timer_adjust_configure(true);
    }
    else
    {
        // Local timer is behind peer: cut next cycle short
        m_params.high_freq_timer[0]->CC[2] = timer_offset;
        ppi_sync_timer_adjust_configure(false);
    }

    APP_ERROR_CHECK_BOOL(m_params.high_freq_timer[0]->CC[2] < TIME_SYNC_TIMER_MAX_VAL);

    nrf_atomic_flag_set(&m_timer_update_in_progress);
    nrf_atomic_u32_fetch_store(&mp_curr_adj_pkt, (uint32_t) p_pkt);

    nrf_atomic_u32_store(&m_curr_adj_timer, m_params.high_freq_timer[0]->CC[1]);
    nrf_atomic_u32_store(&m_curr_adj_counter, m_params.high_freq_timer[1]->CC[1]);

    ppi_sync_timer_adjust_enable();

    return true;
}

static void ppi_sync_timer_adjust_configure(bool do_count)
{
    uint32_t chn0, chn1, chg;

    chn0 = m_params.ppi_chns[0];
    chn1 = m_params.ppi_chns[1];
    chg  = m_params.ppi_chg;

    // PPI channel 0: clear timer when compare[2] value is reached
    NRF_PPI->CHENCLR        = (1 << chn0);
    NRF_PPI->CH[chn0].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2];
    NRF_PPI->CH[chn0].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CLEAR;
    if (do_count)
    {
        NRF_PPI->FORK[chn0].TEP = (uint32_t) &m_params.high_freq_timer[1]->TASKS_COUNT;
    }
    else
    {
        NRF_PPI->FORK[chn0].TEP = 0;
    }

    // PPI channel 1: disable PPI channel 0 such that the timer is only reset once, and trigger software interrupt
    NRF_PPI->CHENCLR        = (1 << chn1);
    NRF_PPI->CH[chn1].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2];
    NRF_PPI->CH[chn1].TEP   = (uint32_t) &NRF_PPI->TASKS_CHG[chg].DIS;	// disable PPI group
    NRF_PPI->FORK[chn1].TEP = (uint32_t) &m_params.egu->TASKS_TRIGGER[0]; // trigger EGU interrupt

    NRF_PPI->TASKS_CHG[chg].DIS = 1;
    NRF_PPI->CHG[chg]           = (1 << chn0) | (1 << chn1);
}

static void ppi_radio_rx_configure(void)
{
    uint32_t chn;

    chn = m_params.ppi_chns[2];

    NRF_PPI->CH[chn].EEP   = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[chn].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1];
    NRF_PPI->FORK[chn].TEP = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[1];
    NRF_PPI->CHENSET       = (1 << chn);
}

static void ppi_radio_tx_configure(void)
{
    uint32_t chn;

    chn = m_params.ppi_chns[0];

    NRF_PPI->CH[chn].EEP   = (uint32_t) &NRF_RADIO->EVENTS_READY;
    NRF_PPI->CH[chn].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1];
    NRF_PPI->FORK[chn].TEP = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[1];
    NRF_PPI->CHENSET       = (1 << chn);
}

static void ppi_timestamp_timer_configure(void)
{
    uint32_t chn;

    chn = m_params.ppi_chns[3];

    NRF_PPI->CH[chn].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[0];
    NRF_PPI->CH[chn].TEP   = (uint32_t) &m_params.high_freq_timer[1]->TASKS_COUNT;
    NRF_PPI->FORK[chn].TEP = 0;
    NRF_PPI->CHENSET       = (1 << chn);
}

static void ppi_counter_timer_capture_configure(uint32_t chn)
{
    NRF_PPI->CH[chn].EEP    = (uint32_t) &m_params.egu->EVENTS_TRIGGERED[1];
    NRF_PPI->CH[chn].TEP    = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[3];
    NRF_PPI->FORK[chn].TEP  = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[0];
    NRF_PPI->CHENSET        = (1 << chn);
}

static void ppi_counter_timer_triggered_capture_configure(nrf_ppi_channel_t chn[2], uint32_t eep)
{
    NRF_PPI->CH[chn[0]].EEP    = eep;
    NRF_PPI->CH[chn[0]].TEP    = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[5];
    NRF_PPI->FORK[chn[0]].TEP  = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[5];

    NRF_PPI->CH[chn[1]].EEP    = eep;
    NRF_PPI->CH[chn[1]].TEP    = (uint32_t) &m_params.egu->TASKS_TRIGGER[5];
    NRF_PPI->FORK[chn[1]].TEP  = 0;

    NRF_PPI->CHENSET        = (1 << chn[0]) | (1 << chn[1]);
}

static void ppi_counter_timer_capture_disable(uint32_t chn)
{
    NRF_PPI->CHENCLR = (1 << chn);

    NRF_PPI->CH[chn].EEP    = 0;
    NRF_PPI->CH[chn].TEP    = 0;
    NRF_PPI->FORK[chn].TEP  = 0;
}

static void ppi_radio_rx_disable(void)
{
    uint32_t chn;

    chn = m_params.ppi_chns[2];

    NRF_PPI->CHENCLR = (1 << chn);
}

static void ppi_sync_timer_adjust_enable(void)
{
    NRF_PPI->TASKS_CHG[m_params.ppi_chg].EN = 1;
}

// static void ppi_sync_timer_adjust_disable(void)
// {
//     NRF_PPI->TASKS_CHG[m_params.ppi_chg].DIS = 1;
// }

uint32_t ppi_sync_trigger_configure(uint32_t ppi_endpoint)
{
    uint32_t error;

    error = nrfx_ppi_channel_assign(m_params.ppi_chns[4], (uint32_t) &m_params.high_freq_timer[1]->EVENTS_COMPARE[4], ppi_endpoint);
    if (error != NRF_SUCCESS)
    {
        return error;
    }

    return nrfx_ppi_channel_fork_assign(m_params.ppi_chns[4], (uint32_t) &m_params.egu->TASKS_TRIGGER[2]);
}

static void timers_capture(uint32_t * p_sync_timer_val, uint32_t * p_count_timer_val, int32_t * p_peer_counter_diff)
{
    static nrf_atomic_flag_t m_timestamp_capture_flag = 0;

    volatile int32_t peer_counter_diff;

    if (nrf_atomic_flag_set_fetch(&m_timestamp_capture_flag) != 0)
    {
        // Not thread-safe
        APP_ERROR_CHECK_BOOL(false);
    }

    nrf_ppi_channel_t ppi_chn;
    nrfx_err_t ret = nrfx_ppi_channel_alloc(&ppi_chn);
    APP_ERROR_CHECK_BOOL(ret == NRFX_SUCCESS);

    ppi_counter_timer_capture_configure(ppi_chn);

    // Loop if adjustment procedure happened close to timer capture
    do
    {
        NVIC_DisableIRQ(m_params.egu_irq_type);

        m_params.egu->EVENTS_TRIGGERED[1] = 0;
        m_params.egu->TASKS_TRIGGER[1] = 1;
        while (m_params.egu->EVENTS_TRIGGERED[1] == 0)
        {
            __NOP();
        }

        peer_counter_diff = m_master_counter_diff;

        NVIC_EnableIRQ(m_params.egu_irq_type);
    } while (m_params.high_freq_timer[0]->CC[3] < 2);


    ppi_counter_timer_capture_disable(ppi_chn);
    nrfx_ppi_channel_free(ppi_chn);

    *p_sync_timer_val = m_params.high_freq_timer[0]->CC[3];
    *p_count_timer_val = (m_params.high_freq_timer[1]->CC[0]);
    *p_peer_counter_diff = peer_counter_diff;

    nrf_atomic_flag_clear(&m_timestamp_capture_flag);
}

uint32_t ts_init(const ts_init_t * p_init)
{
    if (p_init->high_freq_timer[0] == 0 ||
        p_init->high_freq_timer[1] == 0 ||
        p_init->egu                == 0 ||
        p_init->evt_handler        == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(m_params.high_freq_timer, p_init->high_freq_timer, sizeof(m_params.high_freq_timer));
    m_params.egu = p_init->egu;
    m_params.egu_irq_type = p_init->egu_irq_type;
    m_callback = p_init->evt_handler;

    nrfx_err_t ret = nrfx_ppi_group_alloc(&m_params.ppi_chg);
    if (ret != NRFX_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    for (size_t i = 0; i < sizeof(m_params.ppi_chns) / sizeof(m_params.ppi_chns[0]); i++)
    {
        ret = nrfx_ppi_channel_alloc(&m_params.ppi_chns[i]);
        if (ret != NRFX_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    if (m_params.egu != NRF_EGU3)
    {
        // TODO: Remove hardcoded use of SWI3_EGU3_IRQHandler()
        return NRF_ERROR_INVALID_PARAM;
    }

    // TODO: Implement use of RTC as a low-power (and lower accuracy)
    // alternative to 16 MHz TIMER
    // if (SYNC_RTC_PRESCALER != m_params.rtc->PRESCALER)
    // {
    //     // TODO: Handle this
    //     return NRF_ERROR_INVALID_STATE;
    // }

    return NRF_SUCCESS;
}

uint32_t ts_enable(const ts_rf_config_t* p_rf_config)
{
    uint32_t err_code;

    if (p_rf_config == NULL || p_rf_config->rf_addr == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (m_timeslot_session_open)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    err_code = sd_clock_hfclk_request();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code |= sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    memcpy(m_params.rf_addr, p_rf_config->rf_addr, sizeof(m_params.rf_addr));
    m_params.rf_chn = p_rf_config->rf_chn;

    err_code = sd_radio_session_open(radio_callback);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = sd_radio_request(&m_timeslot_req_earliest);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ppi_timestamp_timer_configure();
    // ppi_sync_timer_adjust_configure();

    NVIC_ClearPendingIRQ(m_params.egu_irq_type);
    NVIC_SetPriority(m_params.egu_irq_type, TIME_SYNC_EVT_HANDLER_IRQ_PRIORITY);
    NVIC_EnableIRQ(m_params.egu_irq_type);

    m_params.egu->INTENCLR = 0xFFFFFFFF;
    m_params.egu->INTENSET = EGU_INTENSET_TRIGGERED0_Msk | EGU_INTENSET_TRIGGERED2_Msk | EGU_INTENSET_TRIGGERED3_Msk | EGU_INTENSET_TRIGGERED4_Msk | EGU_INTENSET_TRIGGERED5_Msk;

    m_blocked_cancelled_count  = 0;
    m_radio_state              = RADIO_STATE_IDLE;

    nrf_atomic_flag_clear(&m_send_sync_pkt);

    timestamp_counter_start();
    sync_timer_start();

    nrf_atomic_flag_set(&m_timeslot_session_open);

    return NRF_SUCCESS;
}

uint32_t ts_disable(void)
{
    uint32_t err_code;

    nrf_atomic_flag_set(&m_pending_close);

    err_code = sd_radio_session_close();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // TODO: stop timer

    err_code = sd_clock_hfclk_release();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code |= sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // TODO:
    //       - Close SoftDevice radio session (sd_radio_session_close())
    //       - Stop radio activity
    //       - Stop timers
    //       - Disable used PPI channels
    //       - Disable used interrupts
    //       - Release HFCLK (sd_clock_hfclk_release()),
    //       - Go back to low-power (mode sd_power_mode_set(NRF_POWER_MODE_LOWPWR))
    // Care must be taken to ensure clean stop. Order of tasks above should be reconsidered.
    return NRF_SUCCESS;
}

uint32_t ts_tx_start(uint32_t sync_freq_hz)
{
    uint32_t distance;

    if (sync_freq_hz == TIME_SYNC_FREQ_AUTO)
    {
        // 20-30 Hz is a good range
        // Higher frequency gives more margin for missed packets, but doesn't improve accuracy
        uint32_t auto_freq_target = 30;
        distance = (ROUNDED_DIV(ROUNDED_DIV(1000000, auto_freq_target), (TIME_SYNC_TIMER_MAX_VAL / 16))) * (TIME_SYNC_TIMER_MAX_VAL / 16);
    }
    else
    {
        distance = (1000000 / sync_freq_hz);
    }

    if (distance >= NRF_RADIO_DISTANCE_MAX_US)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_timeslot_distance = distance;
    m_usec_since_tx_offset_calc = 0;

    nrf_atomic_flag_set(&m_send_sync_pkt);

    return NRF_SUCCESS;
}

uint32_t ts_tx_stop()
{
    nrf_atomic_flag_clear(&m_send_sync_pkt);

    return NRF_SUCCESS;
}

uint32_t ts_timestamp_get_ticks_u32(void)
{
    uint32_t sync_timer_val;
    uint32_t count_timer_val;
    int32_t peer_diff;

    timers_capture(&sync_timer_val, &count_timer_val, &peer_diff);

    return (((count_timer_val + peer_diff) * TIME_SYNC_TIMER_MAX_VAL) + sync_timer_val);
}

uint64_t ts_timestamp_get_ticks_u64(void)
{
    uint32_t sync_timer_val;
    uint32_t count_timer_val;
    uint64_t timestamp;
    int32_t  peer_diff;

    timers_capture(&sync_timer_val, &count_timer_val, &peer_diff);

    timestamp  = (uint64_t) count_timer_val;
    timestamp += (uint64_t) peer_diff;
    timestamp *= TIME_SYNC_TIMER_MAX_VAL;
    timestamp += (uint64_t) sync_timer_val;

    return timestamp;
}
