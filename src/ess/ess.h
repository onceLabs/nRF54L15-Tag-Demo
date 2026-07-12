/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Bluetooth SIG Environmental Sensing Service (ESS, 0x181A).
 */
#ifndef APP_ESS_H_
#define APP_ESS_H_

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_APP_ESS)

/**
 * @brief Start periodic ESS notifications.
 *
 * The GATT service itself is registered statically at boot; this starts the
 * work that refreshes and notifies the characteristics.
 *
 * @return 0 on success, negative errno otherwise.
 */
int ess_init(void);

#else

static inline int ess_init(void)
{
	return 0;
}

#endif /* CONFIG_APP_ESS */

#endif /* APP_ESS_H_ */
