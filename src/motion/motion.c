/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Custom BLE Motion Service (128-bit vendor UUID).
 *
 * Two notify characteristics — accelerometer and gyroscope — stream the
 * high-rate IMU samples produced by the sensor module. A consumer thread
 * drains imu_stream_q (via sensors_stream_get), packs however many samples
 * are queued (1..N, capped by the ATT MTU) into each notification, and
 * notifies whichever characteristic(s) a client has subscribed. Sampling
 * auto-starts when the first characteristic is subscribed and stops when the
 * last unsubscribes; a short connection interval is requested on subscribe.
 *
 * Wire format per notification (little-endian):
 *   [uint8 count][ count x { int32 x, int32 y, int32 z, uint32 t_us } ]
 *   accel = micro-m/s^2, gyro = micro-rad/s, t_us = uptime microseconds.
 */
#include "motion.h"
#include "sensors.h"
#include "ble_core.h"

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_motion, CONFIG_LOG_DEFAULT_LEVEL);

/* 128-bit vendor UUIDs: base f0de1b00-9b0f-4a3e-8b1a-2f9c0d5e7a10. */
#define BT_UUID_TAG_MOTION_SVC_VAL \
	BT_UUID_128_ENCODE(0xf0de1b00, 0x9b0f, 0x4a3e, 0x8b1a, 0x2f9c0d5e7a10)
#define BT_UUID_TAG_MOTION_ACCEL_VAL \
	BT_UUID_128_ENCODE(0xf0de1b01, 0x9b0f, 0x4a3e, 0x8b1a, 0x2f9c0d5e7a10)
#define BT_UUID_TAG_MOTION_GYRO_VAL \
	BT_UUID_128_ENCODE(0xf0de1b02, 0x9b0f, 0x4a3e, 0x8b1a, 0x2f9c0d5e7a10)
#define BT_UUID_TAG_MOTION_CFG_VAL \
	BT_UUID_128_ENCODE(0xf0de1b03, 0x9b0f, 0x4a3e, 0x8b1a, 0x2f9c0d5e7a10)

#define BT_UUID_TAG_MOTION_SVC   BT_UUID_DECLARE_128(BT_UUID_TAG_MOTION_SVC_VAL)
#define BT_UUID_TAG_MOTION_ACCEL BT_UUID_DECLARE_128(BT_UUID_TAG_MOTION_ACCEL_VAL)
#define BT_UUID_TAG_MOTION_GYRO  BT_UUID_DECLARE_128(BT_UUID_TAG_MOTION_GYRO_VAL)
#define BT_UUID_TAG_MOTION_CFG   BT_UUID_DECLARE_128(BT_UUID_TAG_MOTION_CFG_VAL)

/* Value-attribute indices within motion_svc (see the layout below). */
#define ATTR_ACCEL_VAL 2
#define ATTR_GYRO_VAL  5

/* Config characteristic write-command field IDs. */
enum motion_cfg_field {
	MOTION_CFG_POLL_RATE  = 0, /* stream poll rate Hz (0 = track ODR) */
	MOTION_CFG_ACCEL_ODR  = 1, /* imu-accel ODR Hz */
	MOTION_CFG_GYRO_ODR   = 2, /* imu-gyro ODR Hz  */
	MOTION_CFG_ACCEL_RANGE = 3, /* imu-accel full-scale G   */
	MOTION_CFG_GYRO_RANGE  = 4, /* imu-gyro full-scale dps  */
};

#define SAMPLE_WIRE_SIZE 16              /* 3x int32 + uint32 */
#define MAX_SAMPLES_PER_NOTIF 30
#define NOTIF_BUF_SIZE (1 + MAX_SAMPLES_PER_NOTIF * SAMPLE_WIRE_SIZE)

static bool accel_subscribed;
static bool gyro_subscribed;

static void accel_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void gyro_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* Config: read the current settings as a 20-byte little-endian struct. */
static ssize_t cfg_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint8_t cfg[20];

	sys_put_le32(sensors_stream_get_rate(), &cfg[0]);
	sys_put_le32(sensors_get_odr(MOTION_IMU_ACCEL), &cfg[4]);
	sys_put_le32(sensors_get_odr(MOTION_IMU_GYRO), &cfg[8]);
	sys_put_le32(sensors_get_range(MOTION_IMU_ACCEL), &cfg[12]);
	sys_put_le32(sensors_get_range(MOTION_IMU_GYRO), &cfg[16]);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, cfg, sizeof(cfg));
}

/* Config: write one [uint8 field_id][uint32 value] command. */
static ssize_t cfg_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 const void *buf, uint16_t len, uint16_t offset,
			 uint8_t flags)
{
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != 5) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *p = buf;
	uint8_t field = p[0];
	uint32_t value = sys_get_le32(&p[1]);
	int err;

	switch (field) {
	case MOTION_CFG_POLL_RATE:
		err = sensors_stream_set_rate(value);
		break;
	case MOTION_CFG_ACCEL_ODR:
		err = sensors_set_odr(MOTION_IMU_ACCEL, value);
		break;
	case MOTION_CFG_GYRO_ODR:
		err = sensors_set_odr(MOTION_IMU_GYRO, value);
		break;
	case MOTION_CFG_ACCEL_RANGE:
		err = sensors_set_range(MOTION_IMU_ACCEL, value);
		break;
	case MOTION_CFG_GYRO_RANGE:
		err = sensors_set_range(MOTION_IMU_GYRO, value);
		break;
	default:
		LOG_WRN("cfg write: unknown field %u", field);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (err) {
		LOG_WRN("cfg write: field %u value %u rejected (%d)", field,
			value, err);
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	LOG_INF("cfg write: field %u = %u", field, value);
	return len;
}

BT_GATT_SERVICE_DEFINE(motion_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_TAG_MOTION_SVC),

	BT_GATT_CHARACTERISTIC(BT_UUID_TAG_MOTION_ACCEL, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(accel_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_TAG_MOTION_GYRO, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(gyro_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_TAG_MOTION_CFG,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       cfg_read, cfg_write, NULL),
);

static void request_fast_interval(void)
{
	struct bt_conn *conn = ble_peripheral_conn();

	if (!conn) {
		return;
	}

	struct bt_le_conn_param param = {
		.interval_min = CONFIG_APP_MOTION_CONN_INTERVAL_MIN,
		.interval_max = CONFIG_APP_MOTION_CONN_INTERVAL_MAX,
		.latency = 0,
		.timeout = 400, /* 4 s supervision timeout */
	};

	int err = bt_conn_le_param_update(conn, &param);

	if (err) {
		LOG_WRN("conn param update request failed (%d)", err);
	}
}

static void subscription_changed(void)
{
	static bool streaming;
	bool any = accel_subscribed || gyro_subscribed;

	if (any && !streaming) {
		if (sensors_stream_start() == 0) {
			streaming = true;
			request_fast_interval();
			LOG_INF("motion: streaming started (accel=%d gyro=%d)",
				accel_subscribed, gyro_subscribed);
		}
	} else if (!any && streaming) {
		sensors_stream_stop();
		streaming = false;
		LOG_INF("motion: streaming stopped");
	}
}

static void accel_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	accel_subscribed = (value == BT_GATT_CCC_NOTIFY);
	subscription_changed();
}

static void gyro_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	gyro_subscribed = (value == BT_GATT_CCC_NOTIFY);
	subscription_changed();
}

static void pack_sample(uint8_t *buf, uint8_t idx, int32_t x, int32_t y,
			int32_t z, uint32_t t_us)
{
	uint8_t *p = &buf[1 + idx * SAMPLE_WIRE_SIZE];

	sys_put_le32((uint32_t)x, p);
	sys_put_le32((uint32_t)y, p + 4);
	sys_put_le32((uint32_t)z, p + 8);
	sys_put_le32(t_us, p + 12);
}

static int notify_chan(uint8_t attr_idx, uint8_t *buf, uint8_t count)
{
	buf[0] = count;

	int err = bt_gatt_notify(NULL, &motion_svc.attrs[attr_idx], buf,
				 1 + count * SAMPLE_WIRE_SIZE);

	if (err == -ENOMEM) {
		/* TX buffers momentarily full: brief backoff + one retry. */
		k_sleep(K_MSEC(1));
		err = bt_gatt_notify(NULL, &motion_svc.attrs[attr_idx], buf,
				     1 + count * SAMPLE_WIRE_SIZE);
	}

	return err;
}

static uint8_t mtu_sample_cap(void)
{
	struct bt_conn *conn = ble_peripheral_conn();
	uint16_t mtu = conn ? bt_gatt_get_mtu(conn) : 23;
	uint32_t cap = (mtu > 4) ? (mtu - 3 - 1) / SAMPLE_WIRE_SIZE : 1;

	if (cap < 1) {
		cap = 1;
	}
	if (cap > MAX_SAMPLES_PER_NOTIF) {
		cap = MAX_SAMPLES_PER_NOTIF;
	}
	return (uint8_t)cap;
}

static void motion_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	static uint8_t accel_buf[NOTIF_BUF_SIZE];
	static uint8_t gyro_buf[NOTIF_BUF_SIZE];

	while (1) {
		if (!(accel_subscribed || gyro_subscribed) ||
		    !sensors_stream_active()) {
			k_sleep(K_MSEC(200));
			continue;
		}

		struct imu_sample s;

		/* Block for the first sample, then greedily pack whatever else
		 * is queued up to the MTU cap.
		 */
		if (sensors_stream_get(&s, K_MSEC(100)) != 0) {
			continue;
		}

		uint8_t cap = mtu_sample_cap();
		uint8_t n = 0;

		do {
			pack_sample(accel_buf, n, s.ax, s.ay, s.az, s.t_us);
			pack_sample(gyro_buf, n, s.gx, s.gy, s.gz, s.t_us);
			n++;
		} while (n < cap && sensors_stream_get(&s, K_NO_WAIT) == 0);

		if (accel_subscribed) {
			notify_chan(ATTR_ACCEL_VAL, accel_buf, n);
		}
		if (gyro_subscribed) {
			notify_chan(ATTR_GYRO_VAL, gyro_buf, n);
		}
	}
}

K_THREAD_DEFINE(motion_tid, 2048, motion_thread, NULL, NULL, NULL, 6, 0, 0);

int motion_init(void)
{
	LOG_INF("Motion Service ready");
	return 0;
}
