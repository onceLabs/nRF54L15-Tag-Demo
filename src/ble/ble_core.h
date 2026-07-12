/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Bluetooth core module: stack enable, connection & security handling.
 */
#ifndef APP_BLE_CORE_H_
#define APP_BLE_CORE_H_

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>

#if IS_ENABLED(CONFIG_APP_BLE_CORE)

/**
 * @brief Enable the Bluetooth stack and register shared callbacks.
 *
 * Tracks up to two links by GAP role and drives the status LED on connect.
 * The outbound central link (CS reflector) is elevated to encrypted (L2)
 * security — Channel Sounding requires it; the inbound peripheral link
 * (screen / phone / SMP host) is left unencrypted so it need not pair.
 *
 * @return 0 on success, negative errno otherwise.
 */
int ble_core_init(void);

/**
 * @brief The outbound central connection (our CS reflector link), or NULL.
 *
 * The returned reference is owned by the module; do not unref it.
 */
struct bt_conn *ble_central_conn(void);

/**
 * @brief The inbound peripheral connection (screen / phone / SMP host), or NULL.
 *
 * The returned reference is owned by the module; do not unref it.
 */
struct bt_conn *ble_peripheral_conn(void);

#else

static inline int ble_core_init(void)
{
	return 0;
}

#endif /* CONFIG_APP_BLE_CORE */

#endif /* APP_BLE_CORE_H_ */
