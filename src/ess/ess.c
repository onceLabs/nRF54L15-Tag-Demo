/*
 * Bluetooth SIG Environmental Sensing Service (ESS, 0x181A).
 *
 * Exposes the BME688 temperature/humidity/pressure as the standard SIG
 * characteristics (read + notify). Values are sourced from the sensor
 * module's cached reading (sensors_get_env) and pushed on a periodic work.
 * Modeled on the Zephyr peripheral_esp sample; no ESS library exists.
 *
 * SIG encodings (little-endian, fixed-point):
 *   Temperature 0x2A6E : sint16, 0.01 degC   -> degC  * 100
 *   Humidity    0x2A6F : uint16, 0.01 %RH    -> %RH   * 100
 *   Pressure    0x2A6D : uint32, 0.1 Pa      -> kPa   * 10000
 */
#include "ess.h"
#include "sensors.h"

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_ess, CONFIG_LOG_DEFAULT_LEVEL);

/* Value-attribute indices within ess_svc (see the service layout below). */
#define ESS_ATTR_TEMP  2
#define ESS_ATTR_HUMID 5
#define ESS_ATTR_PRESS 8

/* Pre-scaled characteristic values, host byte order. */
static int16_t temp_val;
static uint16_t humid_val;
static uint32_t press_val;

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint16_t value = sys_cpu_to_le16(*(const uint16_t *)attr->user_data);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
				 sizeof(value));
}

static ssize_t read_u32(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint32_t value = sys_cpu_to_le32(*(const uint32_t *)attr->user_data);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
				 sizeof(value));
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	LOG_DBG("ESS CCC changed: 0x%04x", value);
}

BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_u16, NULL, &temp_val),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_u16, NULL, &humid_val),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_PRESSURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_u32, NULL, &press_val),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void notify_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(notify_work, notify_work_handler);

static void notify_work_handler(struct k_work *work)
{
	struct env_reading env;

	if (sensors_get_env(&env) == 0) {
		temp_val = (int16_t)(sensor_value_to_double(&env.temp) * 100.0);
		humid_val = (uint16_t)(sensor_value_to_double(&env.humidity) * 100.0);
		press_val = (uint32_t)(sensor_value_to_double(&env.press) * 10000.0);

		uint16_t t_le = sys_cpu_to_le16((uint16_t)temp_val);
		uint16_t h_le = sys_cpu_to_le16(humid_val);
		uint32_t p_le = sys_cpu_to_le32(press_val);

		/* NULL conn = notify all subscribed; -ENOTCONN when none. */
		bt_gatt_notify(NULL, &ess_svc.attrs[ESS_ATTR_TEMP], &t_le,
			       sizeof(t_le));
		bt_gatt_notify(NULL, &ess_svc.attrs[ESS_ATTR_HUMID], &h_le,
			       sizeof(h_le));
		bt_gatt_notify(NULL, &ess_svc.attrs[ESS_ATTR_PRESS], &p_le,
			       sizeof(p_le));
	}

	k_work_reschedule(&notify_work,
			  K_MSEC(CONFIG_APP_ESS_NOTIFY_INTERVAL_MS));
}

int ess_init(void)
{
	LOG_INF("ESS ready");
	k_work_reschedule(&notify_work,
			  K_MSEC(CONFIG_APP_ESS_NOTIFY_INTERVAL_MS));
	return 0;
}
