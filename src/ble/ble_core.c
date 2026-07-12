/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Bluetooth core module: stack enable, connection & security handling.
 *
 * Role-agnostic plumbing shared by the CS initiator/reflector modules:
 *   - enables the stack (and loads settings when bonding is configured),
 *   - tracks up to two links by GAP role: the outbound central link (the CS
 *     reflector, exposed via ble_central_conn()) and an inbound peripheral link
 *     (a screen / phone / SMP host, via ble_peripheral_conn()),
 *   - elevates the central link to encrypted (L2) security (required by CS);
 *     the peripheral link is left unencrypted so a screen/SMP host need not pair,
 *   - drives the status LED on connect/disconnect (unless the UX module owns
 *     the RGB LED, i.e. CONFIG_APP_UX).
 *
 * Scanning/advertising and the CS handshake live in the role modules, which
 * register their own connection callbacks alongside these.
 */
#include "ble_core.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(app_ble_core, CONFIG_LOG_DEFAULT_LEVEL);

/* Two links, disambiguated by GAP role: the central link is our outbound CS
 * reflector connection; the peripheral link is an inbound screen/phone/SMP host.
 */
static struct bt_conn *central_conn;
static struct bt_conn *peripheral_conn;

struct bt_conn *ble_central_conn(void)
{
	return central_conn;
}

struct bt_conn *ble_peripheral_conn(void)
{
	return peripheral_conn;
}

#if !IS_ENABLED(CONFIG_APP_UX)
/* The UX module (when present) owns the RGB LED, incl. led0/led1_blue. */
static const struct gpio_dt_spec status_led =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

static void led_set(bool on)
{
	if (status_led.port) {
		gpio_pin_set_dt(&status_led, on ? 1 : 0);
	}
}
#else
static inline void led_set(bool on)
{
	ARG_UNUSED(on);
}
#endif

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_WRN("Failed to connect to %s (0x%02x)", addr, err);
		return;
	}

	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info)) {
		LOG_WRN("bt_conn_get_info failed for %s", addr);
		return;
	}

	led_set(true);

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		/* Outbound CS reflector link — Channel Sounding needs encryption. */
		LOG_INF("Connected (central/reflector): %s", addr);
		central_conn = bt_conn_ref(conn);

		err = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (err) {
			LOG_WRN("Failed to set security L2 (%d)", err);
		}
	} else {
		/* Inbound screen/phone/SMP link — left unencrypted (no forced pair). */
		LOG_INF("Connected (peripheral/host): %s", addr);
		peripheral_conn = bt_conn_ref(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	if (conn == central_conn) {
		bt_conn_unref(central_conn);
		central_conn = NULL;
	} else if (conn == peripheral_conn) {
		bt_conn_unref(peripheral_conn);
		peripheral_conn = NULL;
	}

	led_set(central_conn != NULL || peripheral_conn != NULL);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_WRN("Security failed: %s level %u err %d", addr, level, err);
	} else {
		LOG_INF("Security changed: %s level %u", addr, level);
	}
}

BT_CONN_CB_DEFINE(ble_core_conn_cbs) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

int ble_core_init(void)
{
	int err;

#if !IS_ENABLED(CONFIG_APP_UX)
	if (status_led.port) {
		if (!gpio_is_ready_dt(&status_led)) {
			LOG_WRN("status LED not ready");
		} else {
			gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
		}
	}
#endif

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	LOG_INF("Bluetooth initialized");
	return 0;
}
