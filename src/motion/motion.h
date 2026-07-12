/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Custom BLE Motion Service: high-rate accel + gyro streaming via notifications.
 */
#ifndef APP_MOTION_H_
#define APP_MOTION_H_

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_APP_MOTION)

/**
 * @brief Initialise the Motion Service consumer.
 *
 * The GATT service is registered statically at boot; streaming starts when a
 * client subscribes to the accel and/or gyro characteristic.
 *
 * @return 0 on success.
 */
int motion_init(void);

#else

static inline int motion_init(void)
{
	return 0;
}

#endif /* CONFIG_APP_MOTION */

#endif /* APP_MOTION_H_ */
