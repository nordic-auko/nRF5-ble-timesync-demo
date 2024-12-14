/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/shell/shell.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <nrfx_gpiote.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

#include "time_sync.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static bool m_gpio_trigger_enabled;
static bool m_send_sync_pkt;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(NRF_DT_GPIOTE_INST(DT_ALIAS(syncpin), gpios));
static const struct gpio_dt_spec syncpin = GPIO_DT_SPEC_GET(DT_ALIAS(syncpin), gpios);
static nrfx_gpiote_pin_t syncpin_absval;

#if defined(DPPI_PRESENT)
static uint8_t dppi_channel_syncpin;
#endif

static void button_changed(uint32_t button_state, uint32_t has_changed);

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received %d bytes from: %s", len, addr);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

static void ts_gpio_trigger_enable(void)
{
	uint64_t time_now_ticks;
	uint32_t time_now_msec;
	uint32_t time_target;
	int err;

	if (m_gpio_trigger_enabled) {
		return;
	}

	// Round up to nearest second to next 1000 ms to start toggling.
	// If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.

	time_now_ticks = ts_timestamp_get_ticks_u64();
	time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

	time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (1000 * 2);
	time_target = (time_target / 1000) * 1000;

#if defined(DPPI_PRESENT)
	err = ts_set_trigger(time_target, dppi_channel_syncpin);
	__ASSERT_NO_MSG(err == 0);
#else
	err = ts_set_trigger(time_target, nrfx_gpiote_out_task_address_get(&gpiote_inst, syncpin_absval));
	__ASSERT_NO_MSG(err == 0);
#endif

	nrfx_gpiote_set_task_trigger(&gpiote_inst, syncpin_absval);

	m_gpio_trigger_enabled = true;
}

static void ts_gpio_trigger_disable(void)
{
	m_gpio_trigger_enabled = false;
}

static void ts_event_handler(const ts_evt_t* evt)
{
	switch (evt->type)
	{
		case TS_EVT_SYNCHRONIZED:
			ts_gpio_trigger_enable();
			LOG_INF("TS_EVT_SYNCHRONIZED");
			break;
		case TS_EVT_DESYNCHRONIZED:
			ts_gpio_trigger_disable();
			LOG_INF("TS_EVT_DESYNCHRONIZED");
			break;
		case TS_EVT_TRIGGERED:
			if (m_gpio_trigger_enabled)
			{
				uint32_t tick_target;
				int err;

				/* Small increments here are more sensitive to drift.
				 * That is, an update from the timing transmitter that causes a jump larger than the
				 * chosen increment, risk having a trigger target_tick that is in the past.
				 */
				tick_target = evt->params.triggered.tick_target + 2;

#if defined(DPPI_PRESENT)
				err = ts_set_trigger(tick_target, dppi_channel_syncpin);
				__ASSERT_NO_MSG(err == 0);
#else
				err = ts_set_trigger(tick_target, nrfx_gpiote_out_task_address_get(&gpiote_inst, syncpin_absval));
				__ASSERT_NO_MSG(err == 0);
#endif

			}
			else
			{
				// Ensure pin is low when triggering is stopped
				nrfx_gpiote_clr_task_trigger(&gpiote_inst, syncpin_absval);
			}
			break;
		default:
			__ASSERT_NO_MSG(false);
			break;
	}
}


static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;
	int err;

	if (buttons & DK_BTN1_MSK) {
		if (m_send_sync_pkt) 		{
			m_send_sync_pkt = false;
			m_gpio_trigger_enabled = false;

			err = ts_tx_stop();
			__ASSERT_NO_MSG(err == 0);

			dk_set_led_off(DK_LED1);

			LOG_INF("Stopping sync beacon transmission!");
		}
		else {
			m_send_sync_pkt = true;

			err = ts_tx_start(TIME_SYNC_FREQ_AUTO);
			__ASSERT_NO_MSG(err == 0);

			dk_set_led_on(DK_LED1);

			ts_gpio_trigger_enable();

			LOG_INF("Starting sync beacon transmission!");
		}
	}
}

static void configure_sync_timer(void)
{
	int err;

	err = ts_init(ts_event_handler);
	__ASSERT_NO_MSG(err == 0);

	ts_rf_config_t rf_config = {
		.rf_chn = 80,
		.rf_addr = { 0xDE, 0xAD, 0xBE, 0xEF, 0x19 }
	};

	err = ts_enable(&rf_config);
	__ASSERT_NO_MSG(err == 0);
}

static nrfx_gpiote_pin_t pin_absval_get(const struct gpio_dt_spec *gpio_spec)
{
	if (gpio_spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio0))) {
		return NRF_GPIO_PIN_MAP(0, gpio_spec->pin);
	}
#if DT_NODE_EXISTS(DT_NODELABEL(gpio1))
	if (gpio_spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio1))) {
		return NRF_GPIO_PIN_MAP(1, gpio_spec->pin);
	}
#endif

	__ASSERT(false, "Port could not be determined");
	return 0;
}

static void configure_debug_gpio(void)
{
	nrfx_err_t nrfx_err;
	int err;


	nrfx_gpiote_output_config_t gpiote_cfg = {
		.drive = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull = NRF_GPIO_PIN_NOPULL,
	};

	nrfx_gpiote_task_config_t task_cfg = {
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
	};

	syncpin_absval = pin_absval_get(&syncpin);

	err = gpio_pin_configure_dt(&syncpin, GPIO_OUTPUT_LOW | syncpin.dt_flags);
	__ASSERT_NO_MSG(err == 0);

	if (!nrfx_gpiote_init_check(&gpiote_inst)) {
		nrfx_err = nrfx_gpiote_init(&gpiote_inst, 5);
		__ASSERT_NO_MSG(nrfx_err == NRFX_SUCCESS);
	}

	nrfx_err = nrfx_gpiote_channel_alloc(&gpiote_inst, &task_cfg.task_ch);
	__ASSERT_NO_MSG(nrfx_err == NRFX_SUCCESS);

	nrfx_err = nrfx_gpiote_output_configure(&gpiote_inst, syncpin_absval, &gpiote_cfg, &task_cfg);
	__ASSERT_NO_MSG(nrfx_err == NRFX_SUCCESS);

	nrfx_gpiote_out_task_enable(&gpiote_inst, syncpin_absval);

#if defined(DPPI_PRESENT)
	nrfx_err = nrfx_dppi_channel_alloc(&dppi_channel_syncpin);
	__ASSERT_NO_MSG(nrfx_err == NRFX_SUCCESS);

	nrf_gpiote_subscribe_set(
		NRF_GPIOTE, nrfx_gpiote_out_task_get(syncpin_absval), dppi_channel_syncpin);

	nrf_dppi_channels_enable(NRF_DPPIC_NS, BIT(dppi_channel_syncpin));
#endif
}

int main(void)
{
	int err = 0;

	configure_gpio();
	configure_debug_gpio();
	configure_sync_timer();

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
				  ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}
}

#if CONFIG_SHELL

static int cmd_tx_toggle(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	if (m_send_sync_pkt) {
		m_send_sync_pkt = false;
		m_gpio_trigger_enabled = false;

		err = ts_tx_stop();
		__ASSERT_NO_MSG(err == 0);

		dk_set_led_off(DK_LED1);

		shell_print(sh, "Stopping sync beacon transmission!");
	}
	else {
		m_send_sync_pkt = true;

		err = ts_tx_start(TIME_SYNC_FREQ_AUTO);
		__ASSERT_NO_MSG(err == 0);

		dk_set_led_on(DK_LED1);

		ts_gpio_trigger_enable();

		shell_print(sh, "Starting sync beacon transmission!");
	}

	return 0;
}

static int cmd_time_get(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t timestamp_ticks;
	uint32_t timestamp_msec;

	timestamp_ticks = ts_timestamp_get_ticks_u64();
	timestamp_msec = TIME_SYNC_TIMESTAMP_TO_USEC(timestamp_ticks) / 1000;

	shell_print(sh, "%d msec", timestamp_msec);

	return 0;
}

SHELL_CMD_ARG_REGISTER(tx_toggle, NULL, "Time sync TX toggle",
			   cmd_tx_toggle, 0, 0);

SHELL_CMD_ARG_REGISTER(time_get, NULL, "Time sync time get",
			   cmd_time_get, 0, 0);
#endif /* CONFIG_SHELL*/
