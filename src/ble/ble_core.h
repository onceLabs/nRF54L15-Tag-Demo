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
 * Elevates every connection to encrypted (L2) security — Channel Sounding
 * requires an encrypted link — and drives the status LED on connect.
 *
 * @return 0 on success, negative errno otherwise.
 */
int ble_core_init(void);

/**
 * @brief Current active connection, or NULL if disconnected.
 *
 * The returned reference is owned by the module; do not unref it.
 */
struct bt_conn *ble_current_conn(void);

#else

static inline int ble_core_init(void)
{
	return 0;
}

#endif /* CONFIG_APP_BLE_CORE */

#endif /* APP_BLE_CORE_H_ */
