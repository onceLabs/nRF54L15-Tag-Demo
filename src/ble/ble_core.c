/*
 * Bluetooth core module: stack enable, connection & security handling.
 *
 * Role-agnostic plumbing shared by the CS initiator/reflector modules:
 *   - enables the stack (and loads settings when bonding is configured),
 *   - elevates each connection to encrypted (L2) security (required by CS),
 *   - tracks the current connection and exposes it via ble_current_conn(),
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

static struct bt_conn *current_conn;

struct bt_conn *ble_current_conn(void)
{
	return current_conn;
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

	LOG_INF("Connected: %s", addr);
	current_conn = bt_conn_ref(conn);
	led_set(true);

	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		LOG_WRN("Failed to set security L2 (%d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	led_set(false);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
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
