/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Connectable BLE advertising — role-independent.
 *
 * Owns the single connectable advertiser so the tag is discoverable for the
 * phone-facing services (ESS / Motion / SMP) whenever it can accept a
 * connection, not only while it is a CS reflector.
 */
#ifndef APP_ADV_H_
#define APP_ADV_H_

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_APP_ADV)

/** @brief Start the advertiser and perform the initial refresh. */
int adv_init(void);

/**
 * @brief Re-evaluate and reconcile advertising state.
 *
 * Call after anything that changes whether/how the tag should advertise:
 * connection up/down, or a CS role/running change. Idempotent.
 */
void adv_refresh(void);

#else

static inline int adv_init(void)
{
	return 0;
}

static inline void adv_refresh(void)
{
}

#endif /* CONFIG_APP_ADV */

#endif /* APP_ADV_H_ */
