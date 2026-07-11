/*
 * Onboard sensor sampling module.
 *
 *   env0   = BME688 environmental (bosch,bme680 driver) — temp/press/humidity/gas
 *   accel0 = ADXL367 accelerometer                       — accel XYZ
 *   imu0   = BMI270 IMU                                   — accel XYZ + gyro XYZ
 *
 * A dedicated thread binds the devices, configures the IMU, then samples and
 * logs every CONFIG_APP_SENSORS_SAMPLE_INTERVAL_MS. Sensors that fail to bind
 * are skipped so a single missing device does not stop the others.
 */
#include "sensors.h"

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_sensors, CONFIG_LOG_DEFAULT_LEVEL);

#define SAMPLE_INTERVAL K_MSEC(CONFIG_APP_SENSORS_SAMPLE_INTERVAL_MS)

#define SENSOR_THREAD_STACK_SIZE 2048
#define SENSOR_THREAD_PRIO       7

static const struct device *const env_dev   = DEVICE_DT_GET(DT_ALIAS(env0));
static const struct device *const accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct device *const imu_dev   = DEVICE_DT_GET(DT_ALIAS(imu0));

static bool env_ok;
static bool accel_ok;
static bool imu_ok;

/* Serializes bus access between the sampling thread and the config path
 * (shell today, BLE later), so a config write can't interleave with a read.
 */
static K_MUTEX_DEFINE(sensor_mtx);

/* Latest environmental reading, cached under sensor_mtx for consumers (ESS). */
static struct env_reading cached_env;

/* Per-peripheral log gate. Written from the shell thread, read from the
 * sensor thread; a single-word bool is atomic on Cortex-M. Sampling is
 * unaffected — only the data log line is suppressed.
 */
static bool log_enabled[SENSOR_ID_COUNT] = { true, true, true };

static const char *const sensor_names[SENSOR_ID_COUNT] = {
	[SENSOR_ID_ENV] = "env",
	[SENSOR_ID_ACCEL] = "accel",
	[SENSOR_ID_IMU] = "imu",
};

int sensors_set_log(enum sensor_id id, bool enable)
{
	if (id < 0 || id >= SENSOR_ID_COUNT) {
		return -EINVAL;
	}
	log_enabled[id] = enable;
	return 0;
}

bool sensors_log_enabled(enum sensor_id id)
{
	if (id < 0 || id >= SENSOR_ID_COUNT) {
		return false;
	}
	return log_enabled[id];
}

const char *sensors_id_name(enum sensor_id id)
{
	if (id < 0 || id >= SENSOR_ID_COUNT) {
		return "?";
	}
	return sensor_names[id];
}

int sensors_get_env(struct env_reading *out)
{
	int ret;

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	if (cached_env.valid) {
		*out = cached_env;
		ret = 0;
	} else {
		ret = -EAGAIN;
	}
	k_mutex_unlock(&sensor_mtx);

	return ret;
}

/* --- Stationarity monitor (ADXL367 accel-magnitude variance) -------------- */

static atomic_t m_stationary; /* 0 = moving/unknown, 1 = stationary */

bool sensors_is_stationary(void)
{
	return atomic_get(&m_stationary) != 0;
}

/* Read the ADXL367 acceleration in m/s^2 under the bus mutex. */
static int read_accel_ms2(float v[3])
{
	struct sensor_value acc[3];
	int err;

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	err = sensor_sample_fetch(accel_dev);
	if (!err) {
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	}
	k_mutex_unlock(&sensor_mtx);

	if (err) {
		return err;
	}

	v[0] = (float)sensor_value_to_double(&acc[0]);
	v[1] = (float)sensor_value_to_double(&acc[1]);
	v[2] = (float)sensor_value_to_double(&acc[2]);
	return 0;
}

static void stationarity_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	static float mags[CONFIG_APP_SENSORS_STATIONARY_WINDOW];
	uint8_t count = 0;
	uint8_t idx = 0;
	const k_timeout_t period =
		K_MSEC(1000 / CONFIG_APP_SENSORS_STATIONARY_RATE_HZ);
	const float thr = CONFIG_APP_SENSORS_STATIONARY_THRESH_MMS2 / 1000.0f;

	while (1) {
		float v[3];

		if (!device_is_ready(accel_dev) || read_accel_ms2(v) != 0) {
			k_sleep(K_MSEC(500));
			continue;
		}

		mags[idx] = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		idx = (idx + 1) % CONFIG_APP_SENSORS_STATIONARY_WINDOW;
		if (count < CONFIG_APP_SENSORS_STATIONARY_WINDOW) {
			count++;
		}

		if (count >= CONFIG_APP_SENSORS_STATIONARY_WINDOW) {
			float mean = 0.0f;

			for (uint8_t i = 0; i < count; i++) {
				mean += mags[i];
			}
			mean /= count;

			float var = 0.0f;

			for (uint8_t i = 0; i < count; i++) {
				float d = mags[i] - mean;

				var += d * d;
			}
			var /= count;

			atomic_set(&m_stationary, (var < thr * thr) ? 1 : 0);
		}

		k_sleep(period);
	}
}

K_THREAD_DEFINE(stationarity_tid, 1024, stationarity_thread, NULL, NULL, NULL,
		8, 0, 0);

/* --- Runtime ODR / range configuration ------------------------------------ */

static const struct {
	const struct device *dev;
	enum sensor_channel chan;
	const char *name;
} motion_map[MOTION_CHAN_COUNT] = {
	[MOTION_ACCEL]     = { DEVICE_DT_GET(DT_ALIAS(accel0)),
			       SENSOR_CHAN_ACCEL_XYZ, "accel" },
	[MOTION_IMU_ACCEL] = { DEVICE_DT_GET(DT_ALIAS(imu0)),
			       SENSOR_CHAN_ACCEL_XYZ, "imu-accel" },
	[MOTION_IMU_GYRO]  = { DEVICE_DT_GET(DT_ALIAS(imu0)),
			       SENSOR_CHAN_GYRO_XYZ, "imu-gyro" },
};

/* Last programmed values, for read-back by status/BLE. ADXL367 range is fixed
 * at its devicetree default (2 G).
 */
static uint32_t motion_odr[MOTION_CHAN_COUNT];
static uint32_t motion_range[MOTION_CHAN_COUNT] = {
	[MOTION_ACCEL] = 2,
};

const char *sensors_motion_name(enum motion_chan ch)
{
	if (ch < 0 || ch >= MOTION_CHAN_COUNT) {
		return "?";
	}
	return motion_map[ch].name;
}

static bool odr_valid(enum motion_chan ch, uint32_t hz)
{
	switch (ch) {
	case MOTION_ACCEL: /* ADXL367 discrete ODRs */
		return hz == 12 || hz == 13 || hz == 25 || hz == 50 ||
		       hz == 100 || hz == 200 || hz == 400;
	case MOTION_IMU_ACCEL:
		return hz >= 1 && hz <= 1600;
	case MOTION_IMU_GYRO:
		return hz >= 25 && hz <= 3200;
	default:
		return false;
	}
}

static bool range_valid(enum motion_chan ch, uint32_t range)
{
	switch (ch) {
	case MOTION_IMU_ACCEL:
		return range == 2 || range == 4 || range == 8 || range == 16;
	case MOTION_IMU_GYRO:
		return range == 125 || range == 250 || range == 500 ||
		       range == 1000 || range == 2000;
	default:
		return false;
	}
}

int sensors_set_odr(enum motion_chan ch, uint32_t hz)
{
	if (ch < 0 || ch >= MOTION_CHAN_COUNT) {
		return -EINVAL;
	}
	if (!odr_valid(ch, hz)) {
		return -EINVAL;
	}

	struct sensor_value v = { .val1 = (int32_t)hz, .val2 = 0 };
	int err;

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	err = sensor_attr_set(motion_map[ch].dev, motion_map[ch].chan,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (!err) {
		motion_odr[ch] = hz;
	}
	k_mutex_unlock(&sensor_mtx);

	return err;
}

uint32_t sensors_get_odr(enum motion_chan ch)
{
	if (ch < 0 || ch >= MOTION_CHAN_COUNT) {
		return 0;
	}
	return motion_odr[ch];
}

int sensors_set_range(enum motion_chan ch, uint32_t range)
{
	if (ch < 0 || ch >= MOTION_CHAN_COUNT) {
		return -EINVAL;
	}
	if (ch == MOTION_ACCEL) {
		/* ADXL367 range is fixed at devicetree init. */
		return -ENOTSUP;
	}
	if (!range_valid(ch, range)) {
		return -EINVAL;
	}

	struct sensor_value v = { .val1 = (int32_t)range, .val2 = 0 };
	int err;

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	err = sensor_attr_set(motion_map[ch].dev, motion_map[ch].chan,
			      SENSOR_ATTR_FULL_SCALE, &v);
	if (!err) {
		motion_range[ch] = range;
	}
	k_mutex_unlock(&sensor_mtx);

	return err;
}

uint32_t sensors_get_range(enum motion_chan ch)
{
	if (ch < 0 || ch >= MOTION_CHAN_COUNT) {
		return 0;
	}
	return motion_range[ch];
}

/* --- High-rate IMU-accel streaming ---------------------------------------- */

#define STREAM_QUEUE_DEPTH 128

K_MSGQ_DEFINE(imu_stream_q, sizeof(struct imu_sample), STREAM_QUEUE_DEPTH, 4);

static atomic_t stream_active;
static atomic_t stream_dropped;
static atomic_t stream_produced;     /* samples generated (for rate stats) */
static uint32_t stream_rate_hz;      /* configured poll rate; 0 = track ODR */
static uint32_t stream_measured_hz;  /* last measured effective rate */

int sensors_stream_set_rate(uint32_t hz)
{
	if (hz && hz > sensors_get_odr(MOTION_IMU_ACCEL)) {
		LOG_WRN("stream rate %u Hz exceeds imu-accel ODR %u Hz "
			"(samples will repeat)", hz,
			sensors_get_odr(MOTION_IMU_ACCEL));
	}
	stream_rate_hz = hz;
	return 0;
}

uint32_t sensors_stream_get_rate(void)
{
	return stream_rate_hz;
}

bool sensors_stream_active(void)
{
	return atomic_get(&stream_active) != 0;
}

int sensors_stream_start(void)
{
	if (!imu_ok) {
		return -ENODEV;
	}
	k_msgq_purge(&imu_stream_q);
	atomic_set(&stream_dropped, 0);
	atomic_set(&stream_produced, 0);
	stream_measured_hz = 0;
	atomic_set(&stream_active, 1);
	return 0;
}

int sensors_stream_stop(void)
{
	atomic_set(&stream_active, 0);
	return 0;
}

int sensors_stream_get(struct imu_sample *out, k_timeout_t timeout)
{
	return k_msgq_get(&imu_stream_q, out, timeout);
}

uint32_t sensors_stream_rate_hz(void)
{
	return stream_measured_hz;
}

uint32_t sensors_stream_dropped(void)
{
	return (uint32_t)atomic_get(&stream_dropped);
}

/* Read one accel + gyro sample from the IMU under the bus mutex.
 * The single fetch reads both channels, so gyro is essentially free.
 */
static int imu_read_motion(struct imu_sample *s)
{
	struct sensor_value acc[3], gyr[3];
	int err;

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	err = sensor_sample_fetch(imu_dev);
	if (!err) {
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
	}
	k_mutex_unlock(&sensor_mtx);

	if (err) {
		return err;
	}

	s->ax = acc[0].val1 * 1000000 + acc[0].val2;
	s->ay = acc[1].val1 * 1000000 + acc[1].val2;
	s->az = acc[2].val1 * 1000000 + acc[2].val2;
	s->gx = gyr[0].val1 * 1000000 + gyr[0].val2;
	s->gy = gyr[1].val1 * 1000000 + gyr[1].val2;
	s->gz = gyr[2].val1 * 1000000 + gyr[2].val2;
	s->t_us = (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
	return 0;
}

/* Producer: paces to the configured poll rate and enqueues accel samples. */
static void imu_stream_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (!sensors_stream_active()) {
			k_sleep(K_MSEC(200));
			continue;
		}

		int64_t deadline = k_uptime_ticks();

		while (sensors_stream_active()) {
			uint32_t rate = stream_rate_hz ? stream_rate_hz :
				sensors_get_odr(MOTION_IMU_ACCEL);

			if (rate == 0) {
				rate = 100;
			}

			deadline += k_us_to_ticks_ceil64(1000000ULL / rate);
			k_sleep(K_TIMEOUT_ABS_TICKS(deadline));

			struct imu_sample s;

			if (imu_read_motion(&s) == 0) {
				atomic_inc(&stream_produced);
				if (k_msgq_put(&imu_stream_q, &s, K_NO_WAIT) != 0) {
					atomic_inc(&stream_dropped);
				}
			}
		}
	}
}

/* Stats: reports the effective sampling rate once per second from the
 * produced counter. It does NOT drain the queue — the motion (BLE) module
 * is the sole consumer via sensors_stream_get().
 */
static void imu_stats_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint32_t last_produced = 0;
	int64_t window_start = k_uptime_get();

	while (1) {
		if (!sensors_stream_active()) {
			k_sleep(K_MSEC(200));
			last_produced = (uint32_t)atomic_get(&stream_produced);
			window_start = k_uptime_get();
			continue;
		}

		k_sleep(K_MSEC(1000));

		int64_t now = k_uptime_get();
		int64_t elapsed = now - window_start;
		uint32_t produced = (uint32_t)atomic_get(&stream_produced);

		if (elapsed > 0) {
			stream_measured_hz =
				(uint32_t)(((uint64_t)(produced - last_produced) *
					    1000) / elapsed);
			LOG_INF("imu stream: ~%u Hz, dropped %u",
				stream_measured_hz, sensors_stream_dropped());
		}
		last_produced = produced;
		window_start = now;
	}
}

K_THREAD_DEFINE(imu_stream_tid, 2048, imu_stream_thread, NULL, NULL, NULL,
		5, 0, 0);
K_THREAD_DEFINE(imu_stats_tid, 2048, imu_stats_thread, NULL, NULL, NULL,
		7, 0, 0);

static void imu_configure(void)
{
	struct sensor_value oversampling = { .val1 = 1, .val2 = 0 };

	/* Normal oversampling (not runtime-exposed). */
	sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);
	sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);

	/* Defaults funneled through the runtime setters so stored state matches
	 * hardware. Range before ODR — the driver treats ODR as the last write
	 * that also selects the power mode.
	 */
	sensors_set_range(MOTION_IMU_ACCEL, 2);   /* +/-2 G   */
	sensors_set_range(MOTION_IMU_GYRO, 500);  /* +/-500 dps */
	sensors_set_odr(MOTION_IMU_ACCEL, 100);
	sensors_set_odr(MOTION_IMU_GYRO, 100);
}

static void sample_env(void)
{
	struct sensor_value temp, press, humidity, gas;
	int err;

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	err = sensor_sample_fetch(env_dev);
	if (!err) {
		sensor_channel_get(env_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(env_dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(env_dev, SENSOR_CHAN_HUMIDITY, &humidity);
		sensor_channel_get(env_dev, SENSOR_CHAN_GAS_RES, &gas);
	}
	if (!err) {
		cached_env.temp = temp;
		cached_env.press = press;
		cached_env.humidity = humidity;
		cached_env.valid = true;
	}
	k_mutex_unlock(&sensor_mtx);

	if (err) {
		LOG_WRN("bme688 fetch failed (%d)", err);
		return;
	}

	if (!log_enabled[SENSOR_ID_ENV]) {
		return;
	}

	LOG_INF("bme688: T=%.2f C  P=%.2f kPa  RH=%.2f %%  gas=%.0f ohm",
		sensor_value_to_double(&temp), sensor_value_to_double(&press),
		sensor_value_to_double(&humidity), sensor_value_to_double(&gas));
}

static void sample_accel(void)
{
	struct sensor_value acc[3];
	int err;

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	err = sensor_sample_fetch(accel_dev);
	if (!err) {
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	}
	k_mutex_unlock(&sensor_mtx);

	if (err) {
		LOG_WRN("adxl367 fetch failed (%d)", err);
		return;
	}

	if (!log_enabled[SENSOR_ID_ACCEL]) {
		return;
	}

	LOG_INF("adxl367: aX=%.3f aY=%.3f aZ=%.3f (m/s^2)",
		sensor_value_to_double(&acc[0]), sensor_value_to_double(&acc[1]),
		sensor_value_to_double(&acc[2]));
}

static void sample_imu(void)
{
	struct sensor_value acc[3], gyr[3];
	int err;

	/* The high-rate stream thread owns the IMU while streaming. */
	if (sensors_stream_active()) {
		return;
	}

	k_mutex_lock(&sensor_mtx, K_FOREVER);
	err = sensor_sample_fetch(imu_dev);
	if (!err) {
		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
	}
	k_mutex_unlock(&sensor_mtx);

	if (err) {
		LOG_WRN("bmi270 fetch failed (%d)", err);
		return;
	}

	if (!log_enabled[SENSOR_ID_IMU]) {
		return;
	}

	LOG_INF("bmi270: aX=%.3f aY=%.3f aZ=%.3f (m/s^2)  "
		"gX=%.3f gY=%.3f gZ=%.3f (rad/s)",
		sensor_value_to_double(&acc[0]), sensor_value_to_double(&acc[1]),
		sensor_value_to_double(&acc[2]), sensor_value_to_double(&gyr[0]),
		sensor_value_to_double(&gyr[1]), sensor_value_to_double(&gyr[2]));
}

static void sensor_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	env_ok = device_is_ready(env_dev);
	accel_ok = device_is_ready(accel_dev);
	imu_ok = device_is_ready(imu_dev);

	LOG_INF("sensors ready: bme688=%d adxl367=%d bmi270=%d",
		env_ok, accel_ok, imu_ok);

	if (accel_ok) {
		/* Establish a known ODR so status/BLE read-back is accurate
		 * (ADXL367 range stays at its devicetree default).
		 */
		sensors_set_odr(MOTION_ACCEL, 100);
	}

	if (imu_ok) {
		imu_configure();
	}

	while (1) {
		if (env_ok) {
			sample_env();
		}
		if (accel_ok) {
			sample_accel();
		}
		if (imu_ok) {
			sample_imu();
		}
		k_sleep(SAMPLE_INTERVAL);
	}
}

K_THREAD_DEFINE(sensor_tid, SENSOR_THREAD_STACK_SIZE, sensor_thread,
		NULL, NULL, NULL, SENSOR_THREAD_PRIO, 0, SYS_FOREVER_MS);

int sensors_init(void)
{
	k_thread_start(sensor_tid);
	return 0;
}
