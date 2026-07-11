/*
 * Onboard sensor sampling module.
 */
#ifndef APP_SENSORS_H_
#define APP_SENSORS_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#if IS_ENABLED(CONFIG_APP_SENSORS)

/** @brief Identifies an onboard peripheral for per-sensor log control. */
enum sensor_id {
	SENSOR_ID_ENV,   /* BME688 environmental */
	SENSOR_ID_ACCEL, /* ADXL367 accelerometer */
	SENSOR_ID_IMU,   /* BMI270 IMU */
	SENSOR_ID_COUNT,
};

/**
 * @brief Bind the onboard sensors and start periodic sampling/logging.
 *
 * @return 0 on success, negative errno otherwise.
 */
int sensors_init(void);

/**
 * @brief Enable or disable the log output of one peripheral.
 *
 * Sampling continues regardless; this only gates the data log line.
 *
 * @return 0 on success, -EINVAL if @p id is out of range.
 */
int sensors_set_log(enum sensor_id id, bool enable);

/** @brief Whether @p id's log output is currently enabled. */
bool sensors_log_enabled(enum sensor_id id);

/** @brief Short name for @p id ("env"/"accel"/"imu"), or "?" if invalid. */
const char *sensors_id_name(enum sensor_id id);

/** @brief Latest cached environmental reading (BME688). */
struct env_reading {
	struct sensor_value temp;     /* degrees C   */
	struct sensor_value press;    /* kPa         */
	struct sensor_value humidity; /* %RH         */
	bool valid;
};

/**
 * @brief Copy the most recent environmental reading.
 *
 * @return 0 on success, -EAGAIN if the sensor has not been sampled yet.
 */
int sensors_get_env(struct env_reading *out);

/**
 * @brief Whether the tag is currently stationary (low accel-magnitude variance).
 *
 * Motion/stationarity indicator derived from the ADXL367; used to adapt the
 * CS distance filter. Not a displacement source.
 */
bool sensors_is_stationary(void);

/**
 * @brief Runtime-configurable motion channels.
 *
 * The IMU's accelerometer and gyroscope are independent channels. This is the
 * config surface reused by the shell today and by the BLE layer later.
 */
enum motion_chan {
	MOTION_ACCEL,     /* ADXL367 accelerometer */
	MOTION_IMU_ACCEL, /* BMI270 accelerometer  */
	MOTION_IMU_GYRO,  /* BMI270 gyroscope      */
	MOTION_CHAN_COUNT,
};

/**
 * @brief Set the output data rate of a motion channel.
 *
 * @param ch  motion channel
 * @param hz  requested ODR in Hz (drivers snap/validate; see limits below)
 * @return 0 on success, -EINVAL for an unsupported value/channel, or a
 *         negative driver error.
 *
 * Limits: ADXL367 accepts {12/13(=12.5), 25, 50, 100, 200, 400};
 *         BMI270 accel <= 1600 Hz, BMI270 gyro <= 3200 Hz (snapped).
 */
int sensors_set_odr(enum motion_chan ch, uint32_t hz);

/** @brief Last ODR (Hz) programmed for @p ch, or 0 if unknown. */
uint32_t sensors_get_odr(enum motion_chan ch);

/**
 * @brief Set the full-scale range of a motion channel.
 *
 * @param ch     motion channel
 * @param range  accel: G {2,4,8,16}; gyro: dps {125,250,500,1000,2000}
 * @return 0 on success, -ENOTSUP for MOTION_ACCEL (ADXL367 range is fixed at
 *         devicetree init), -EINVAL for a bad value, or a driver error.
 */
int sensors_set_range(enum motion_chan ch, uint32_t range);

/** @brief Last range programmed for @p ch (G or dps), or 0 if unknown. */
uint32_t sensors_get_range(enum motion_chan ch);

/** @brief Name for @p ch ("accel"/"imu-accel"/"imu-gyro"), or "?". */
const char *sensors_motion_name(enum motion_chan ch);

/* --- High-rate IMU-accel streaming ---------------------------------------- */

/** @brief One streamed IMU sample: accel (micro-m/s^2) + gyro (micro-rad/s). */
struct imu_sample {
	int32_t ax;
	int32_t ay;
	int32_t az;
	int32_t gx;
	int32_t gy;
	int32_t gz;
	uint32_t t_us;
};

/**
 * @brief Set the stream poll rate in Hz (independent of the sensor ODR).
 *
 * May be <= the ODR; a rate above the ODR just re-reads unchanged samples
 * (logged as a warning). Pass 0 to track the imu-accel ODR.
 *
 * @return 0 always.
 */
int sensors_stream_set_rate(uint32_t hz);

/** @brief Configured poll rate (Hz), or 0 if tracking the ODR. */
uint32_t sensors_stream_get_rate(void);

/** @brief Start high-rate IMU-accel sampling into the stream queue. */
int sensors_stream_start(void);

/** @brief Stop high-rate IMU-accel sampling. */
int sensors_stream_stop(void);

/** @brief Whether the stream is currently active. */
bool sensors_stream_active(void);

/**
 * @brief Pop one sample from the stream queue (consumer API; BLE later).
 *
 * @return 0 on success, -EAGAIN on timeout.
 */
int sensors_stream_get(struct imu_sample *out, k_timeout_t timeout);

/** @brief Last measured effective stream rate (Hz). */
uint32_t sensors_stream_rate_hz(void);

/** @brief Cumulative count of samples dropped because the queue was full. */
uint32_t sensors_stream_dropped(void);

#else

static inline int sensors_init(void)
{
	return 0;
}

static inline bool sensors_is_stationary(void)
{
	return false; /* assume moving -> no adaptive filtering */
}

#endif /* CONFIG_APP_SENSORS */

#endif /* APP_SENSORS_H_ */
