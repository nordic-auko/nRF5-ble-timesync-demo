/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "time_sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <nrfx.h>
#include <nrfx_timer.h>

#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#include <hal/nrf_dppi.h>
#else
#include <nrfx_ppi.h>
#include <hal/nrf_ppi.h>
#endif

#include <hal/nrf_egu.h>
#include <hal/nrf_power.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>

#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>

#include <mpsl_timeslot.h>
#include <mpsl.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#define LOG_MODULE_NAME timesync
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#if   defined ( __CC_ARM )
#define TX_CHAIN_DELAY (699 - 235)
#elif defined ( __ICCARM__ )
#define TX_CHAIN_DELAY (703 - 235)
#elif defined ( __GNUC__ )
#define TX_CHAIN_DELAY (704 - 173)
#endif

#if TIME_SYNC_TIMER_MAX_VAL < 10000
#error Timer values below 10000 not supported
#endif

#define SYNC_PKT_QUEUE_LEN 3

#define TS_LEN_US                            (1000UL)
#define TX_LEN_EXTENSION_US                  (1000UL)
#define TS_SAFETY_MARGIN_US                  (200UL)   /**< The timeslot activity should be finished with this much to spare. */
#define TS_EXTEND_MARGIN_US                  (400UL)   /**< The timeslot activity should request an extension this long before end of timeslot. */

#if defined(CONFIG_SOC_SERIES_NRF53X)
#define TS_SWI_IRQn NRFX_CONCAT_3(SWI, CONFIG_TIMESYNC_SWI, _IRQn)
#define TS_EGU_IRQn NRFX_CONCAT_3(EGU, CONFIG_TIMESYNC_EGU, _IRQn)
#elif defined(CONFIG_SOC_SERIES_NRF52X)
#define TS_SWI_IRQn NRFX_CONCAT_3(SWI, CONFIG_TIMESYNC_SWI, NRFX_CONCAT_3(_EGU, CONFIG_TIMESYNC_SWI, _IRQn))
#define TS_EGU_IRQn NRFX_CONCAT_3(SWI, CONFIG_TIMESYNC_EGU, NRFX_CONCAT_3(_EGU, CONFIG_TIMESYNC_EGU, _IRQn))
#if CONFIG_TIMESYNC_SWI == CONFIG_TIMESYNC_EGU
#error Cannot use the same instance of EGU and SWI
#endif /* CONFIG_TIMESYNC_SWI == CONFIG_TIMESYNC_EGU */
#endif /* defined(CONFIG_SOC_SERIES_NRF53X) */


#if defined(CONFIG_SOC_SERIES_NRF53X)
#define EGU_INST NRF_EGU0
#else
#define EGU_INST NRFX_CONCAT_2(NRF_EGU, CONFIG_TIMESYNC_EGU)
#endif


#define FREEWHEEL_TIMER NRFX_CONCAT_2(NRF_TIMER, CONFIG_TIMESYNC_FREEWHEEL_TIMER)
#define COUNTER_TIMER NRFX_CONCAT_2(NRF_TIMER, CONFIG_TIMESYNC_COUNTER_TIMER)

static void ppi_sync_timer_adjust_configure(bool shorten);
static void ppi_sync_timer_adjust_enable(void);
static void ppi_sync_timer_clear_configure(void);
static void ppi_sync_timer_clear_disable(void);
static void ppi_radio_rx_disable(void);
static void ppi_radio_rx_configure(void);
static void ppi_radio_tx_configure(void);
static int ppi_sync_trigger_configure(uint32_t ppi_endpoint);

struct {
	uint8_t rf_chn;
	uint8_t rf_addr[5];
	#if defined(DPPI_PRESENT)
	uint8_t ppi_chns[7];
	nrf_dppi_channel_group_t ppi_chgs[2];
	#else
	nrf_ppi_channel_t ppi_chns[7];
	nrf_ppi_channel_group_t ppi_chgs[2];
	#endif
} m_params;

#if CONFIG_TIMESYNC_FREEWHEEL_TIMER == CONFIG_TIMESYNC_COUNTER_TIMER
#error Freewheel timer and counter cannot be the same
#endif

__packed struct sync_pkt {
	int32_t  timer_val;
	int32_t  rtc_val;
	uint32_t counter_val;
};

BUILD_ASSERT(sizeof(struct sync_pkt) == 12);

static atomic_t m_timeslot_session_open;
static atomic_t m_pending_close;
static atomic_t m_pending_tx_stop;
static atomic_t m_blocked_cancelled_count;
static uint32_t m_total_timeslot_length = 0;
static uint32_t m_timeslot_distance = 0;
static uint32_t m_tx_slot_retry_count = 0;
static uint32_t m_usec_since_sync_packet = 0;
static uint32_t m_usec_since_tx_offset_calc = 0;

static atomic_t m_send_sync_pkt = false;
static atomic_t m_timer_update_in_progress = false;

static bool m_synchronized = false;

static volatile int64_t m_master_counter_diff = 0;
static atomic_t m_rcv_count = 0;

static atomic_t mp_curr_adj_pkt;
static atomic_t m_curr_adj_timer;
static atomic_t m_curr_adj_counter;

static ts_evt_handler_t m_callback;
static atomic_t m_tick_target;
static atomic_t m_sync_packet_count = 0;
static atomic_t m_used_packet_count = 0;
static atomic_t m_last_sync = 0;
static atomic_t m_prev_sync_pkt_timer;
static atomic_t m_prev_sync_pkt_counter;

#if defined(DPPI_PRESENT)
static uint8_t m_timestamp_trigger_ppi[2];
#else
static nrf_ppi_channel_t m_timestamp_trigger_ppi[2];
#endif

static bool m_timestamp_trigger_set = false;

static struct onoff_manager *clk_mgr;
static struct onoff_client clk_cli;

/* Simple circular buffer: no free, only take */
static uint8_t  m_sync_pkt_ringbuf[10][sizeof(struct sync_pkt)];
static atomic_t m_sync_pkt_ringbuf_idx;

static volatile enum
{
	RADIO_STATE_IDLE, /* Default state */
	RADIO_STATE_RX,   /* Waiting for packets */
	RADIO_STATE_TX    /* Trying to transmit packet */
} m_radio_state = RADIO_STATE_IDLE;

/**< This will be used when requesting the first timeslot or any time a timeslot is blocked or cancelled. */
static mpsl_timeslot_request_t m_timeslot_req_earliest = {
		.request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
		.params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED,
		.params.earliest.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
		.params.earliest.length_us = TS_LEN_US,
		.params.earliest.timeout_us = 100000 /* Expect timeslot within 100 ms */
};

/**< This will be used at the end of each timeslot to request the next timeslot. */
static mpsl_timeslot_request_t m_timeslot_req_normal = {
	.request_type = MPSL_TIMESLOT_REQ_TYPE_NORMAL,
	.params.normal.hfclk = MPSL_TIMESLOT_HFCLK_CFG_XTAL_GUARANTEED,
	.params.normal.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
	.params.normal.distance_us = 0,
	.params.normal.length_us = TS_LEN_US,
};

static mpsl_timeslot_signal_return_param_t signal_callback_return_param;
static mpsl_timeslot_session_id_t session_id;

/* Ring buffer for handling low priority signals outside of timeslot callback */
RING_BUF_DECLARE(callback_low_priority_ring_buf, 10);

static void timeslot_begin_handler(void);
static void timeslot_end_handler(void);
static void timeslot_radio_handler(void);

#if defined(DPPI_PRESENT)
static void ppi_counter_timer_triggered_capture_configure(uint8_t chn[2], uint32_t eep);
#else
static void ppi_counter_timer_triggered_capture_configure(nrf_ppi_channel_t chn[2], uint32_t eep);
#endif
static bool sync_timer_offset_compensate(struct sync_pkt * p_pkt);
static struct sync_pkt * tx_buf_get(void);

static void increment_desync_timeout(uint32_t increment_usec);

ISR_DIRECT_DECLARE(swi_isr)
{
	static const char *signal_type_str[] = {
		"MPSL_TIMESLOT_SIGNAL_START",
		"MPSL_TIMESLOT_SIGNAL_TIMER0",
		"MPSL_TIMESLOT_SIGNAL_RADIO",
		"MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED",
		"MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED",
		"MPSL_TIMESLOT_SIGNAL_BLOCKED",
		"MPSL_TIMESLOT_SIGNAL_CANCELLED",
		"MPSL_TIMESLOT_SIGNAL_SESSION_IDLE",
		"MPSL_TIMESLOT_SIGNAL_INVALID_RETURN",
		"MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED",
		"MPSL_TIMESLOT_SIGNAL_OVERSTAYED",
	};
	uint8_t signal_type = 0;

	while (!ring_buf_is_empty(&callback_low_priority_ring_buf)) {
		if (ring_buf_get(&callback_low_priority_ring_buf, &signal_type, 1) == 1) {
			int err;

			__ASSERT_NO_MSG(signal_type < ARRAY_SIZE(signal_type_str));

			LOG_INF("Signal: %s", signal_type_str[signal_type]);

			switch (signal_type) {
			case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
				err = mpsl_timeslot_session_close(session_id);
				if (err) {
					LOG_ERR("mpsl_timeslot_session_close=%d", err);
				}
				break;
			case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
				atomic_clear(&m_timeslot_session_open);
				atomic_clear(&m_pending_close);
				break;
			case MPSL_TIMESLOT_SIGNAL_BLOCKED:
			case MPSL_TIMESLOT_SIGNAL_CANCELLED:
				if (!m_pending_close)
				{
					int err;
					/* This is typically caused by a conflict with an active BLE session. */
					if (m_send_sync_pkt && m_tx_slot_retry_count < 3)
					{
						/* Try a few times to skip the next scheduled event, else get earliest possible. */
						++m_tx_slot_retry_count;
						m_timeslot_req_normal.params.normal.distance_us = m_timeslot_distance * 2;

						err = mpsl_timeslot_request(
							session_id,
							&m_timeslot_req_normal);
						if (err) {
							LOG_ERR("mpsl_timeslot_request(earliest)=%d", err);
						}
					}
					else
					{
						err = mpsl_timeslot_request(
							session_id,
							&m_timeslot_req_earliest);
						if (err) {
							LOG_ERR("mpsl_timeslot_request(earliest)=%d", err);
						}
					}

					atomic_add(&m_blocked_cancelled_count, 1);
				}
				break;
			default:
				__ASSERT_NO_MSG(false);
				break;
			}
		}
	}

	return 1;
}

ISR_DIRECT_DECLARE(egu_isr)
{
	if (nrf_egu_event_check(EGU_INST, NRF_EGU_EVENT_TRIGGERED0))
	{
		nrf_egu_event_clear(EGU_INST, NRF_EGU_EVENT_TRIGGERED0);

		m_master_counter_diff = ((struct sync_pkt *) mp_curr_adj_pkt)->counter_val - m_curr_adj_counter;
		atomic_add(&m_used_packet_count, 1);
		atomic_set(&m_last_sync, (m_curr_adj_counter - m_master_counter_diff));

		nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL0, TIME_SYNC_TIMER_MAX_VAL);
		nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL2, 0);

		atomic_clear(&m_timer_update_in_progress);

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

	if (nrf_egu_event_check(EGU_INST, NRF_EGU_EVENT_TRIGGERED2))
	{
		nrf_egu_event_clear(EGU_INST, NRF_EGU_EVENT_TRIGGERED2);

		#if defined(DPPI_PRESENT)
		nrf_dppi_channels_disable(NRF_DPPIC,(0x1UL << m_params.ppi_chns[4]));
		#else
		nrf_ppi_channel_disable(NRF_PPI, m_params.ppi_chns[4]);
		#endif


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

	if (nrf_egu_event_check(EGU_INST, NRF_EGU_EVENT_TRIGGERED3))
	{
		nrf_egu_event_clear(EGU_INST, NRF_EGU_EVENT_TRIGGERED3);

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

	if (nrf_egu_event_check(EGU_INST, NRF_EGU_EVENT_TRIGGERED4))
	{
		nrf_egu_event_clear(EGU_INST, NRF_EGU_EVENT_TRIGGERED4);
		// TODO: Remove this event, as it is not currently used
	}

	if (nrf_egu_event_check(EGU_INST, NRF_EGU_EVENT_TRIGGERED5))
	{
		uint64_t timestamp;
		uint32_t counter_val;
		uint32_t timer_val_cc5;
		uint32_t timer_val_cc2;

		nrf_egu_event_clear(EGU_INST, NRF_EGU_EVENT_TRIGGERED5);

		counter_val = nrf_timer_cc_get(COUNTER_TIMER, NRF_TIMER_CC_CHANNEL5);
		timer_val_cc2 = nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL2);
		timer_val_cc5 = nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL5);

		if ((timer_val_cc5 <= 5) || (timer_val_cc5 == TIME_SYNC_TIMER_MAX_VAL) ||
			(timer_val_cc2 != 0 && timer_val_cc5 == timer_val_cc2))
		{
			// Check if counter increment was caught
			nrf_timer_task_trigger(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE5);
			if (nrf_timer_cc_get(COUNTER_TIMER, NRF_TIMER_CC_CHANNEL5) != counter_val)
			{
				counter_val = nrf_timer_cc_get(COUNTER_TIMER, NRF_TIMER_CC_CHANNEL5);
			}
		}

		timestamp  = counter_val;
		timestamp += m_master_counter_diff;
		timestamp *= TIME_SYNC_TIMER_MAX_VAL;
		timestamp += timer_val_cc5;

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

	return 1;
}

static mpsl_timeslot_signal_return_param_t *mpsl_timeslot_callback(
	mpsl_timeslot_session_id_t session_id,
	uint32_t signal_type)
{
	(void) session_id; /* unused parameter */
	uint8_t input_data = (uint8_t)signal_type;
	uint32_t input_data_len;

	mpsl_timeslot_signal_return_param_t *p_ret_val = NULL;

	switch (signal_type) {

	case MPSL_TIMESLOT_SIGNAL_START:
		nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0,
			(TS_LEN_US - TS_SAFETY_MARGIN_US));
		nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL1,
			(TS_LEN_US - TS_EXTEND_MARGIN_US));
		if (m_send_sync_pkt) {
			nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
		} else {
			nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK | NRF_TIMER_INT_COMPARE1_MASK);
		}
		nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
		nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE1);
		nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);

		timeslot_begin_handler();
		break;
	case MPSL_TIMESLOT_SIGNAL_RADIO:
		timeslot_radio_handler();
		break;
	case MPSL_TIMESLOT_SIGNAL_TIMER0:
		if (nrf_timer_event_check(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0)) {

			nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
			nrf_timer_int_disable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

			// This is the "timeslot is about to end" timeout

			// Schedule next timeslot
			if (m_send_sync_pkt)
			{
				m_timeslot_req_normal.params.normal.distance_us = m_timeslot_distance;

				signal_callback_return_param.params.request.p_next =
					&m_timeslot_req_normal;
				signal_callback_return_param.callback_action =
					MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
			}
			else
			{
				signal_callback_return_param.params.request.p_next =
					&m_timeslot_req_earliest;
				signal_callback_return_param.callback_action =
					MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
			}

			timeslot_end_handler();

			p_ret_val = &signal_callback_return_param;
		}

		if (nrf_timer_event_check(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE1)) {

			nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE1);

			// This is the "try to extend timeslot" timeout

			if (m_total_timeslot_length < (128000000UL - 5000UL - TX_LEN_EXTENSION_US) && !m_send_sync_pkt)
			{
				// Request timeslot extension if total length does not exceed 128 seconds

				signal_callback_return_param.params.request.p_next =
					&m_timeslot_req_normal;
				signal_callback_return_param.callback_action =
					MPSL_TIMESLOT_SIGNAL_ACTION_EXTEND;
			}
			else if (!m_send_sync_pkt)
			{
				signal_callback_return_param.params.request.p_next =
					&m_timeslot_req_earliest;
				signal_callback_return_param.callback_action =
					MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
			}
		}
		break;
	case MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED:
		// Extension succeeded: update timer
		{
			uint32_t cc0_val;
			uint32_t cc1_val;

			cc0_val = nrf_timer_cc_get(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0);
			cc1_val = nrf_timer_cc_get(NRF_TIMER0, NRF_TIMER_CC_CHANNEL1);

			nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, cc0_val + TX_LEN_EXTENSION_US - 25);
			nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL1, cc1_val + TX_LEN_EXTENSION_US - 25);
		}

		// Keep track of total length
		m_total_timeslot_length += TX_LEN_EXTENSION_US;

		increment_desync_timeout(TX_LEN_EXTENSION_US);
		break;
	case MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED:
		timeslot_end_handler();
		increment_desync_timeout(TX_LEN_EXTENSION_US);

		p_ret_val = &signal_callback_return_param;
		signal_callback_return_param.params.request.p_next =
			&m_timeslot_req_earliest;
		signal_callback_return_param.callback_action =
			MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
		break;
	case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
	case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
	case MPSL_TIMESLOT_SIGNAL_BLOCKED:
	case MPSL_TIMESLOT_SIGNAL_CANCELLED:
		input_data_len = ring_buf_put(&callback_low_priority_ring_buf, &input_data, 1);
		if (input_data_len != 1) {
			LOG_ERR("Full ring buffer, enqueue data with length %d", input_data_len);
			k_oops();
		}
		break;
	default:
		LOG_ERR("unexpected signal: %u", signal_type);
		k_oops();
		break;
	}

#if defined(CONFIG_SOC_SERIES_NRF53X)
	NVIC_SetPendingIRQ(TS_SWI_IRQn);
#elif defined(CONFIG_SOC_SERIES_NRF52X)
	NVIC_SetPendingIRQ(TS_SWI_IRQn);
#endif

	if (p_ret_val == NULL) {
		signal_callback_return_param.callback_action =
			MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
		signal_callback_return_param.params.request.p_next =
					&m_timeslot_req_normal;
		p_ret_val = &signal_callback_return_param;
	}

	return p_ret_val;
}


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
		nrf_egu_task_trigger(EGU_INST, NRF_EGU_TASK_TRIGGER3);
		m_usec_since_sync_packet = 0; // Reset to suppress needless irq generation
	}
}

static struct sync_pkt * tx_buf_get(void)
{
    uint32_t idx;

    idx = atomic_inc(&m_sync_pkt_ringbuf_idx) % ARRAY_SIZE(m_sync_pkt_ringbuf);

    return (struct sync_pkt *) m_sync_pkt_ringbuf[idx];
}

static void update_radio_parameters(struct sync_pkt * p_pkt)
{
	const nrf_radio_packet_conf_t pkt_conf = {
		.lflen = 0,
		.s0len = 0,
		.s1len = 0,
		.maxlen = sizeof(struct sync_pkt),
		.statlen = sizeof(struct sync_pkt),
		.balen = 4,
		.big_endian = true,
		.whiteen = true,
#if defined(RADIO_PCNF0_PLEN_Msk)
		.plen = NRF_RADIO_PREAMBLE_LENGTH_16BIT,
#endif
#if defined(RADIO_PCNF0_CRCINC_Msk)
		.crcinc = false,
#endif
#if defined(RADIO_PCNF0_S1INCL_Msk)
		.s1incl = false,
#endif
#if defined(RADIO_PCNF0_CILEN_Msk)
		.cilen = 0,
#endif
#if defined(RADIO_PCNF0_TERMLEN_Msk)
		.termlen = 0,
#endif
	};

	nrf_radio_power_set(NRF_RADIO, true);
	nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_0DBM);
	nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_2MBIT);
	nrf_radio_modecnf0_set(NRF_RADIO, true, RADIO_MODECNF0_DTX_Center);
	nrf_radio_crc_configure(NRF_RADIO, 3, NRF_RADIO_CRC_ADDR_INCLUDE, 0x11021UL);
	nrf_radio_crcinit_set(NRF_RADIO, 0xFFFFFFUL);
	nrf_radio_packet_configure(NRF_RADIO, &pkt_conf);
	nrf_radio_packetptr_set(NRF_RADIO, p_pkt);
	nrf_radio_base0_set(NRF_RADIO,
		m_params.rf_addr[1] << 24 | m_params.rf_addr[2] << 16 |
		m_params.rf_addr[3] << 8 | m_params.rf_addr[4]);
	nrf_radio_prefix0_set(NRF_RADIO, m_params.rf_addr[0]);
	nrf_radio_txaddress_set(NRF_RADIO, 0);
	nrf_radio_rxaddresses_set(NRF_RADIO, BIT(0));
	nrf_radio_frequency_set(NRF_RADIO, 2400 + m_params.rf_chn);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
	nrf_radio_int_disable(NRF_RADIO, 0xFFFFFFFF);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_END_MASK);

	NVIC_SetPriority(RADIO_IRQn, MPSL_HIGH_IRQ_PRIORITY);
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);
}

void timeslot_end_handler(void)
{
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	nrf_radio_int_disable(NRF_RADIO, 0xFFFFFFFF);

	ppi_radio_rx_disable();
	nrf_radio_power_set(NRF_RADIO, false);
	NVIC_DisableIRQ(RADIO_IRQn);
	NVIC_ClearPendingIRQ(RADIO_IRQn);

	m_radio_state = RADIO_STATE_IDLE;
}

void timeslot_begin_handler(void)
{
	struct sync_pkt * p_pkt;

	m_total_timeslot_length = 0;
	m_tx_slot_retry_count = 0;

	if (m_pending_tx_stop)
	{
		atomic_clear(&m_send_sync_pkt);
		atomic_clear(&m_pending_tx_stop);
		m_timeslot_distance = 0;
		m_synchronized = false;
	}

	if (!m_send_sync_pkt)
	{
		if (m_radio_state    != RADIO_STATE_RX ||
			nrf_radio_state_get(NRF_RADIO) != NRF_RADIO_STATE_RX)
		{
			p_pkt = tx_buf_get();

			update_radio_parameters(p_pkt);

			nrf_radio_shorts_enable(NRF_RADIO, NRF_RADIO_SHORT_READY_START_MASK);
			nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);

			ppi_radio_rx_configure();

			m_radio_state = RADIO_STATE_RX;
		}

		return;
	}

	if (m_radio_state == RADIO_STATE_RX)
	{
		// Packet transmission has now started (state change from RX).
		nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
		nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
		while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
			__NOP();
		}
	}

	p_pkt = tx_buf_get();

	ppi_radio_tx_configure();
	update_radio_parameters(p_pkt);

	nrf_radio_shorts_enable(NRF_RADIO,
		NRF_RADIO_SHORT_READY_START_MASK | NRF_RADIO_SHORT_END_DISABLE_MASK);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);

	while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_READY)) {
		// PPI is used to trigger sync timer capture when radio is ready
		// Radio will automatically start transmitting once ready, so the captured timer value must be copied into radio packet buffer ASAP
		__NOP();
	}

	p_pkt->timer_val   = nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL1);
	p_pkt->counter_val = nrf_timer_cc_get(COUNTER_TIMER, NRF_TIMER_CC_CHANNEL1);
	p_pkt->counter_val += m_master_counter_diff;

	atomic_add(&m_sync_packet_count, 1);
	m_radio_state = RADIO_STATE_TX;
}

static void timeslot_radio_handler(void)
{

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);

	if (m_radio_state == RADIO_STATE_RX && nrf_radio_crc_status_check(NRF_RADIO)) {
		struct sync_pkt * p_pkt;
		bool         adjustment_procedure_started;

		// LOG_INF("RX");

		p_pkt = (struct sync_pkt *) nrf_radio_packetptr_get(NRF_RADIO);

		if (p_pkt->timer_val <= 2)
		{
			// Ignore packet due to potential missed counter increment
			// TODO: See if this can be tightened
			goto resume_radio_rx;
		}


		adjustment_procedure_started = sync_timer_offset_compensate(p_pkt);
		atomic_add(&m_sync_packet_count, 1);

		if (adjustment_procedure_started)
		{
			atomic_set(&m_prev_sync_pkt_timer, p_pkt->timer_val); // TODO: Necessary?
			atomic_set(&m_prev_sync_pkt_counter, p_pkt->counter_val); // TODO: Necessary?

			p_pkt = tx_buf_get();

			nrf_radio_packetptr_set(NRF_RADIO, p_pkt);

			m_usec_since_sync_packet = 0;
		}

		atomic_add(&m_rcv_count, 1);
	}

resume_radio_rx:

	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_START);
}

static void timestamp_counter_start(void)
{
	// COUNTER_TIMER is used in counter mode to count the number of sync timer overflows/resets
	// When timestamp API is used, the number of overflows/resets + current value of sync timer must be added up to give accurate timestamp information
	nrf_timer_task_trigger(COUNTER_TIMER, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(COUNTER_TIMER, NRF_TIMER_TASK_CLEAR);
	nrf_timer_frequency_set(COUNTER_TIMER, NRF_TIMER_FREQ_16MHz);
	nrf_timer_mode_set(COUNTER_TIMER, NRF_TIMER_MODE_COUNTER);
	nrf_timer_bit_width_set(COUNTER_TIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_task_trigger(COUNTER_TIMER, NRF_TIMER_TASK_START);
}

static void sync_timer_start(void)
{
	// FREEWHEEL_TIMER (NRF_TIMER) is the always-running sync timer
	// The timing master never adjusts this timer
	// The timing slave(s) adjusts this timer whenever a sync packet is received and the logic determines that there is
	nrf_timer_task_trigger(FREEWHEEL_TIMER, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(FREEWHEEL_TIMER, NRF_TIMER_TASK_CLEAR);
	nrf_timer_frequency_set(FREEWHEEL_TIMER, NRF_TIMER_FREQ_16MHz);
	nrf_timer_mode_set(FREEWHEEL_TIMER, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(FREEWHEEL_TIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL0, TIME_SYNC_TIMER_MAX_VAL);
	nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL1, 0xFFFFFFFF);
	nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL2, 0xFFFFFFFF);
	nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL3, 0xFFFFFFFF);
	nrf_timer_task_trigger(FREEWHEEL_TIMER, NRF_TIMER_TASK_START);
}

int ts_set_trigger(uint32_t target_tick, uint32_t ppi_endpoint)
{
	if (!m_timeslot_session_open)
	{
		return -EBUSY;
	}

	if (ppi_sync_trigger_configure(ppi_endpoint) != 0)
	{
		return -EINVAL;
	}

	atomic_set(&m_sync_packet_count, 0);
	atomic_set(&m_used_packet_count, 0);

	// TODO: is there a way to check if the target value is plausible?

	nrf_timer_cc_set(COUNTER_TIMER, NRF_TIMER_CC_CHANNEL4, (target_tick - m_master_counter_diff));
	atomic_set(&m_tick_target, target_tick);

	#if defined(DPPI_PRESENT)
	nrf_dppi_channels_enable(NRF_DPPIC, (0x1UL << m_params.ppi_chns[4])); // activate trigger
	#else
	nrf_ppi_channel_enable(NRF_PPI, m_params.ppi_chns[4]); // activate trigger
	#endif

	return 0;
}

int ts_set_timestamp_trigger(uint32_t ppi_event_endpoint)
{
	// TODO: Check if other timers can be used also
	if (FREEWHEEL_TIMER != NRF_TIMER1 && FREEWHEEL_TIMER != NRF_TIMER2)
	{
		return -EBUSY;
	}
	if (COUNTER_TIMER != NRF_TIMER1 && COUNTER_TIMER != NRF_TIMER2)
	{
		return -EBUSY;
	}

	if (!m_timestamp_trigger_set)
	{
		nrfx_err_t err;

		for (int i = 0; i < ARRAY_SIZE(m_timestamp_trigger_ppi); ++i)
		{
				/* Allocate a (D)PPI channel. */
				#if defined(DPPI_PRESENT)
					err = nrfx_dppi_channel_alloc(&m_timestamp_trigger_ppi[i]);
				#else
					err = nrfx_ppi_channel_alloc(&m_timestamp_trigger_ppi[i]);
				#endif

				if (err != NRFX_SUCCESS)
				{
					return err;
				}
		}
	}

	ppi_counter_timer_triggered_capture_configure(m_timestamp_trigger_ppi, ppi_event_endpoint);

	m_timestamp_trigger_set = true;

	return 0;
}

static inline bool sync_timer_offset_compensate(struct sync_pkt * p_pkt)
{
	uint32_t peer_timer;
	uint32_t local_timer;
	int32_t timer_offset;
	bool wrapped = false;

	if (atomic_set(&m_timer_update_in_progress, true))
	{
		return false;
	}

	peer_timer  = p_pkt->timer_val;
	peer_timer += TX_CHAIN_DELAY;
	if (peer_timer >= TIME_SYNC_TIMER_MAX_VAL)
	{
		peer_timer -= TIME_SYNC_TIMER_MAX_VAL;
		p_pkt->counter_val += 1;
		wrapped = true;
	}

	local_timer = nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL1);
	timer_offset = local_timer - peer_timer;

	if (local_timer > TIME_SYNC_TIMER_MAX_VAL)
	{
		// Todo: Investigate if this is a corner case with a root cause that needs to be handled
		atomic_clear(&m_timer_update_in_progress);
		return false;
	}

	if (timer_offset == TIME_SYNC_TIMER_MAX_VAL || timer_offset == 0)
	{
		// Already in sync
		atomic_add(&m_used_packet_count, 1);
		atomic_clear(&m_timer_update_in_progress);
		return false;
	}

	if (wrapped && (local_timer >= (TIME_SYNC_TIMER_MAX_VAL - 50)))
	{
		// Too close
		// Todo: see if local counter increment can be accounted for
		atomic_clear(&m_timer_update_in_progress);
		return false;
	}

	bool shorten_cycle = timer_offset < 0;

	nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL2, (TIME_SYNC_TIMER_MAX_VAL + timer_offset));
	ppi_sync_timer_adjust_configure(shorten_cycle);

	atomic_set(&mp_curr_adj_pkt, (uint32_t) p_pkt);

	atomic_set(&m_curr_adj_timer, nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL1));
	atomic_set(&m_curr_adj_counter, nrf_timer_cc_get(COUNTER_TIMER, NRF_TIMER_CC_CHANNEL1));

	uint32_t cc_val;

	do
	{
		// Avoid race condition when disabling and re-enabling PPI channel very quickly
		nrf_timer_task_trigger(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE4);
		cc_val = nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL4);
	} while ((cc_val > (TIME_SYNC_TIMER_MAX_VAL - 15)));

	if (shorten_cycle)
	{
		nrf_timer_cc_set(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL0, (TIME_SYNC_TIMER_MAX_VAL + 10));
		ppi_sync_timer_adjust_enable();
	}
	else
	{
		ppi_sync_timer_adjust_enable();
		ppi_sync_timer_clear_disable();
	}

	return true;
}

static void ppi_sync_timer_adjust_configure(bool shorten)
{
	#if defined(DPPI_PRESENT)

	uint8_t  chn0, chn1, chn2;
	nrf_dppi_channel_group_t chg0, chg1;
	#else
		nrf_ppi_channel_t chn0, chn1, chn2;
		nrf_ppi_channel_group_t chg0, chg1;
	#endif

	chn0 = m_params.ppi_chns[0];
	chn1 = m_params.ppi_chns[1];
	chn2 = m_params.ppi_chns[6];
	chg0  = m_params.ppi_chgs[0];
	chg1  = m_params.ppi_chgs[1];

	// PPI channel 0: clear timer when compare[2] value is reached
	#if defined(DPPI_PRESENT)
	nrf_dppi_channels_disable(NRF_DPPIC,(0x1UL << chn0));
	#else
	nrf_ppi_channel_disable(NRF_PPI, chn0);
	#endif



	nrfx_gppi_channel_endpoints_setup(chn0,
		nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE2),
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CLEAR));
	if (shorten)
	{
		nrfx_gppi_fork_endpoint_setup(chn0,
			nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_COUNT));
	}
	else
	{
		nrfx_gppi_fork_endpoint_setup(chn0, 0);
	}

	// PPI channel 1: disable PPI channel 0 such that the timer is only reset once, and trigger software interrupt
	#if defined(DPPI_PRESENT)
	nrf_dppi_channels_disable(NRF_DPPIC,(0x1UL <<chn1));
	#else
	nrf_ppi_channel_disable(NRF_PPI, chn1);
	#endif
	


	#if defined(DPPI_PRESENT)
	nrfx_gppi_channel_endpoints_setup(chn1,
		nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE2),
		nrfx_gppi_group_disable_task_get(chg0));
	nrfx_gppi_fork_endpoint_setup(chn1,nrf_egu_task_address_get(EGU_INST, NRF_EGU_TASK_TRIGGER0));
	#else
		nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn1,
		nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE2),
		nrf_ppi_task_group_disable_address_get(NRF_PPI, chg0),
		nrf_egu_task_address_get(EGU_INST, NRF_EGU_TASK_TRIGGER0));
	#endif

	#if defined(DPPI_PRESENT)
	nrf_dppi_channels_disable(NRF_DPPIC,(0x1UL <<chn2));
	#else
	nrf_ppi_channel_disable(NRF_PPI, chn2);
	#endif

	if (shorten)
	{
		// this asserts, find function to unset ?
		//nrfx_gppi_channel_endpoints_setup( chn2, 0, 0);
	}
	else
	{
			#if defined(DPPI_PRESENT)
			nrfx_gppi_channel_endpoints_setup(chn2,
			nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE2),
				nrfx_gppi_group_disable_task_get(chg1));
			#else
			// PPI channel 2: Re-enable EVENTS_COMPARE[0] after lengthening cycle
			nrf_ppi_channel_endpoint_setup(NRF_PPI, chn2,
				nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE2),
				nrf_ppi_task_group_enable_address_get(NRF_PPI, chg1));
			#endif
	}

	#if defined(DPPI_PRESENT)
		nrf_dppi_group_disable(NRF_DPPIC,chg0);

		nrfx_gppi_channels_include_in_group(
		BIT(chn0) | BIT(chn1) | BIT(chn2),
		chg0);
	#else
		nrf_ppi_group_disable(NRF_PPI, chg0);
		
		nrf_ppi_channels_include_in_group(NRF_PPI,
		BIT(chn0) | BIT(chn1) | BIT(chn2),
		chg0);
	#endif

}

static void ppi_radio_rx_configure(void)
{
	uint32_t chn;

	chn = m_params.ppi_chns[2];

	#if defined(DPPI_PRESENT)
		nrfx_gppi_channel_endpoints_setup(chn,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS),
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE1));
	nrfx_gppi_fork_endpoint_setup(chn,nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE1));

	nrfx_dppi_channel_enable(chn);
	#else
		nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS),
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE1),
		nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE1));

		nrf_ppi_channel_enable(NRF_PPI, chn);
	#endif
}

static void ppi_radio_tx_configure(void)
{
	uint32_t chn;

	chn = m_params.ppi_chns[0];


		#if defined(DPPI_PRESENT)
		nrfx_gppi_channel_endpoints_setup(chn,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_READY),
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE1));
	nrfx_gppi_fork_endpoint_setup(chn,nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE1));

	nrfx_dppi_channel_enable(chn);
	#else
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_READY),
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE1),
		nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE1));
	nrf_ppi_channel_enable(NRF_PPI, chn);
	#endif


}

static void ppi_timestamp_timer_configure(void)
{
	uint32_t chn;

	chn = m_params.ppi_chns[3];




	#if defined(DPPI_PRESENT)
		nrfx_gppi_channel_endpoints_setup(chn,
		nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE0),
		nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_COUNT));

		nrfx_dppi_channel_enable(chn);
	#else
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn,
		nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE0),
		nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_COUNT),
		0);
	nrf_ppi_channel_enable(NRF_PPI, chn);
	#endif


}

static void ppi_counter_timer_capture_configure(uint32_t chn)
{



		#if defined(DPPI_PRESENT)
	nrfx_gppi_channel_endpoints_setup(chn,
		nrf_egu_event_address_get(EGU_INST, NRF_EGU_EVENT_TRIGGERED1),
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE3));
	nrfx_gppi_fork_endpoint_setup(chn,nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE0));
	nrfx_dppi_channel_enable(chn);
	#else
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn,
		nrf_egu_event_address_get(EGU_INST, NRF_EGU_EVENT_TRIGGERED1),
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE3),
		nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE0));
	nrf_ppi_channel_enable(NRF_PPI, chn);
	#endif


}

#if defined(DPPI_PRESENT)
static void ppi_counter_timer_triggered_capture_configure(uint8_t chn[2], uint32_t eep)
{



	nrfx_gppi_channel_endpoints_setup(chn[0],
		eep,
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE5));
	nrfx_gppi_fork_endpoint_setup(chn[0],nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE5));

	nrfx_gppi_channel_endpoints_setup(chn[1],
		eep,
		nrf_egu_task_address_get(EGU_INST, NRF_EGU_TASK_TRIGGER5));

	nrf_dppi_channels_enable(NRF_DPPIC, BIT(chn[0]) | BIT(chn[1]));
}
#else
static void ppi_counter_timer_triggered_capture_configure(nrf_ppi_channel_t chn[2], uint32_t eep)
{

	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn[0],
		eep,
		nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CAPTURE5),
		nrf_timer_task_address_get(COUNTER_TIMER, NRF_TIMER_TASK_CAPTURE5));

	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn[1],
		eep,
		nrf_egu_task_address_get(EGU_INST, NRF_EGU_TASK_TRIGGER5),
		0);

	nrf_ppi_channels_enable(NRF_PPI, BIT(chn[0]) | BIT(chn[1]));
	}
#endif


static void ppi_counter_timer_capture_disable(uint32_t chn)
{
	#if defined(DPPI_PRESENT)
	nrf_dppi_channels_disable(NRF_DPPIC,(0x1UL <<chn));
    
	// this asserts, find function to unset ?
	//nrfx_gppi_channel_endpoints_setup(chn, 0, 0);
	//nrfx_gppi_fork_endpoint_setup(chn,0);


	#else
	nrf_ppi_channel_disable(NRF_PPI, chn);
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, chn, 0, 0, 0);

	#endif

	
}

static void ppi_radio_rx_disable(void)
{
	#if defined(DPPI_PRESENT)
	nrf_dppi_channels_disable(NRF_DPPIC,(0x1UL << m_params.ppi_chns[2]));
	#else
	nrf_ppi_channel_disable(NRF_PPI, m_params.ppi_chns[2]);
	#endif
}

static void ppi_sync_timer_adjust_enable(void)
{
	#if defined(DPPI_PRESENT)
	nrfx_gppi_group_enable(m_params.ppi_chgs[0]);
	#else
		nrf_ppi_group_enable(NRF_PPI, m_params.ppi_chgs[0]);
	#endif



}

static void ppi_sync_timer_clear_configure(void)
{
	uint32_t chn;
	uint32_t chg;

	chn = m_params.ppi_chns[5];
	chg = m_params.ppi_chgs[1];

	#if defined(DPPI_PRESENT)
	nrf_dppi_channels_disable(NRF_DPPIC,(0x1UL << chn));

	nrfx_gppi_channel_endpoints_setup(chn,
	nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE0),
	nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CLEAR));

	nrfx_dppi_channel_enable(chn);

	nrfx_gppi_channels_include_in_group(BIT(chn), chg);
	nrfx_gppi_group_enable(chg);

	#else
	nrf_ppi_channel_disable(NRF_PPI, chn);

	nrf_ppi_channel_endpoint_setup(NRF_PPI, chn,
	nrf_timer_event_address_get(FREEWHEEL_TIMER, NRF_TIMER_EVENT_COMPARE0),
	nrf_timer_task_address_get(FREEWHEEL_TIMER, NRF_TIMER_TASK_CLEAR));

	nrf_ppi_channel_enable(NRF_PPI, chn);

	
	nrf_ppi_channel_include_in_group(NRF_PPI, chn, chg);
	nrf_ppi_group_enable(NRF_PPI, chg);
	#endif

	
}

static void ppi_sync_timer_clear_disable(void)
{
	#if defined(DPPI_PRESENT)
		nrf_dppi_group_disable(NRF_DPPIC,m_params.ppi_chgs[1]);
	#else
		nrf_ppi_group_disable(NRF_PPI, m_params.ppi_chgs[1]);
	#endif
}

int ppi_sync_trigger_configure(uint32_t ppi_endpoint)
{
	#if defined(DPPI_PRESENT)
	nrfx_gppi_channel_endpoints_setup(m_params.ppi_chns[4],
		nrf_timer_event_address_get(COUNTER_TIMER, NRF_TIMER_EVENT_COMPARE4),
		ppi_endpoint);
	nrfx_gppi_fork_endpoint_setup(m_params.ppi_chns[4],nrf_egu_task_address_get(EGU_INST, NRF_EGU_TASK_TRIGGER2));
	#else
	nrf_ppi_channel_and_fork_endpoint_setup(NRF_PPI, m_params.ppi_chns[4],
		nrf_timer_event_address_get(COUNTER_TIMER, NRF_TIMER_EVENT_COMPARE4),
		ppi_endpoint,
		nrf_egu_task_address_get(EGU_INST, NRF_EGU_TASK_TRIGGER2));
	#endif
	return 0;
}

static void timers_capture(uint32_t * p_sync_timer_val, uint32_t * p_count_timer_val, int32_t * p_peer_counter_diff)
{
	static atomic_t m_timestamp_capture_flag = 0;

	volatile int32_t peer_counter_diff;

	if (atomic_set(&m_timestamp_capture_flag, true) != 0)
	{
		// Not thread-safe
		__ASSERT_NO_MSG(false);
	}

	#if defined(DPPI_PRESENT)
	uint8_t  ppi_chn;
	nrfx_err_t ret = nrfx_dppi_channel_alloc(&ppi_chn);
	#else
	nrf_ppi_channel_t ppi_chn;
	nrfx_err_t ret = nrfx_ppi_channel_alloc(&ppi_chn);
	#endif
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);

	ppi_counter_timer_capture_configure(ppi_chn);

	// Loop if adjustment procedure happened close to timer capture
	do
	{
		NVIC_DisableIRQ(TS_EGU_IRQn);

		nrf_egu_event_clear(EGU_INST, NRF_EGU_EVENT_TRIGGERED1);
		nrf_egu_task_trigger(EGU_INST, NRF_EGU_TASK_TRIGGER1);
		while (!nrf_egu_event_check(EGU_INST, NRF_EGU_EVENT_TRIGGERED1))
		{
			__NOP();
		}

		peer_counter_diff = m_master_counter_diff;

		NVIC_EnableIRQ(TS_EGU_IRQn);
	} while (nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL3) < 2);


	ppi_counter_timer_capture_disable(ppi_chn);

	
	#if defined(DPPI_PRESENT)
	nrfx_dppi_channel_free(ppi_chn);
	#else
	nrfx_ppi_channel_free(ppi_chn);
	#endif


	

	*p_sync_timer_val = nrf_timer_cc_get(FREEWHEEL_TIMER, NRF_TIMER_CC_CHANNEL3);
	*p_count_timer_val = nrf_timer_cc_get(COUNTER_TIMER, NRF_TIMER_CC_CHANNEL0);
	*p_peer_counter_diff = peer_counter_diff;

	atomic_clear(&m_timestamp_capture_flag);
}

int ts_init(ts_evt_handler_t evt_handler)
{
	nrfx_err_t ret;

	m_callback = evt_handler;

	
#if defined(DPPI_PRESENT)
	for (size_t i = 0; i < sizeof(m_params.ppi_chgs) / sizeof(m_params.ppi_chgs[0]); i++)
	{
		ret = nrfx_dppi_group_alloc(&m_params.ppi_chgs[i]);
		if (ret != NRFX_SUCCESS)
		{
			return -EBUSY;
		}
	}

	for (size_t i = 0; i < sizeof(m_params.ppi_chns) / sizeof(m_params.ppi_chns[0]); i++)
	{
		ret = nrfx_dppi_channel_alloc(&m_params.ppi_chns[i]);
		if (ret != NRFX_SUCCESS)
		{
			return -EBUSY;
		}
	}

#else
	for (size_t i = 0; i < sizeof(m_params.ppi_chgs) / sizeof(m_params.ppi_chgs[0]); i++)
	{
		ret = nrfx_ppi_group_alloc(&m_params.ppi_chgs[i]);
		if (ret != NRFX_SUCCESS)
		{
			return -EBUSY;
		}
	}

	for (size_t i = 0; i < sizeof(m_params.ppi_chns) / sizeof(m_params.ppi_chns[0]); i++)
	{
		ret = nrfx_ppi_channel_alloc(&m_params.ppi_chns[i]);
		if (ret != NRFX_SUCCESS)
		{
			return -EBUSY;
		}
	}
#endif


	IRQ_DIRECT_CONNECT(TS_SWI_IRQn, 1, swi_isr, 0);
	irq_enable(TS_SWI_IRQn);
	IRQ_DIRECT_CONNECT(TS_EGU_IRQn, TIME_SYNC_EVT_HANDLER_IRQ_PRIORITY, egu_isr, 0);
	irq_enable(TS_EGU_IRQn);

	// TODO: Implement use of RTC as a low-power (and lower accuracy)
	// alternative to 16 MHz TIMER
	// if (SYNC_RTC_PRESCALER != m_params.rtc->PRESCALER)
	// {
	//     // TODO: Handle this
	//     return -EBUSY;
	// }

	return 0;
}

int ts_enable(const ts_rf_config_t* p_rf_config)
{
	int err;

	if (p_rf_config == NULL || p_rf_config->rf_addr == NULL)
	{
		return -EINVAL;
	}

	if (m_timeslot_session_open)
	{
		return -EBUSY;
	}

	/* Enable crystal oscillator for improved clock accuracy */
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		return -ENODEV;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("onoff_request: %d", err);
		return err;
	}

	/* Enable constant latency mode to minimize jitter */
	nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);

	memcpy(m_params.rf_addr, p_rf_config->rf_addr, sizeof(m_params.rf_addr));
	m_params.rf_chn = p_rf_config->rf_chn;

	ppi_timestamp_timer_configure();

	// NVIC_ClearPendingIRQ(EGU_INST_irq_type);
	// NVIC_SetPriority(EGU_INST_irq_type, TIME_SYNC_EVT_HANDLER_IRQ_PRIORITY);
	// NVIC_EnableIRQ(EGU_INST_irq_type);

	nrf_egu_int_disable(EGU_INST, NRF_EGU_INT_ALL);
	nrf_egu_int_enable(EGU_INST,
		NRF_EGU_INT_TRIGGERED0 |
		NRF_EGU_INT_TRIGGERED2 |
		NRF_EGU_INT_TRIGGERED3 |
		NRF_EGU_INT_TRIGGERED4 |
		NRF_EGU_INT_TRIGGERED5);

	m_blocked_cancelled_count  = 0;
	m_radio_state              = RADIO_STATE_IDLE;

	atomic_clear(&m_send_sync_pkt);

	timestamp_counter_start();
	ppi_sync_timer_clear_configure();
	sync_timer_start();

	err = mpsl_timeslot_session_open(
		mpsl_timeslot_callback,
		&session_id);
	if (err) {
		LOG_ERR("mpsl_timeslot_session_open=%d", err);
		return err;
	}

	err = mpsl_timeslot_request(
		session_id,
		&m_timeslot_req_earliest);
	if (err) {
		LOG_ERR("mpsl_timeslot_request(earliest)=%d", err);
		return err;
	}

	atomic_set(&m_timeslot_session_open, true);

	return 0;
}

int ts_disable(void)
{
	int err;

	atomic_set(&m_pending_close, true);

	err = mpsl_timeslot_session_close(session_id);
	if (err) {
		LOG_ERR("mpsl_timeslot_session_close=%d", err);
		return err;
	}

	// TODO: stop timer

	nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_LOWPWR);

	err = onoff_release(clk_mgr);
	if (err < 0) {
		LOG_ERR("onoff_release: %d", err);
		return err;
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
	return 0;
}

int ts_tx_start(uint32_t sync_freq_hz)
{
	uint32_t distance;

	if (sync_freq_hz == TIME_SYNC_FREQ_AUTO)
	{
		// 20-30 Hz is a good range
		// Higher frequency gives more margin for missed packets, but doesn't improve accuracy that much
		uint32_t auto_freq_target = 25;
		distance = (NRFX_ROUNDED_DIV(NRFX_ROUNDED_DIV(1000000, auto_freq_target), (TIME_SYNC_TIMER_MAX_VAL / 16))) * (TIME_SYNC_TIMER_MAX_VAL / 16);
	}
	else
	{
		distance = (1000000 / sync_freq_hz);
	}

	if (distance >= MPSL_TIMESLOT_DISTANCE_MAX_US)
	{
		return -EINVAL;
	}

	m_timeslot_distance = distance;
	m_usec_since_tx_offset_calc = 0;

	atomic_set(&m_send_sync_pkt, true);

	return 0;
}

int ts_tx_stop()
{
	atomic_set(&m_pending_tx_stop, true);

	return 0;
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