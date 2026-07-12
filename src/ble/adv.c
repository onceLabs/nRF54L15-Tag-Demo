/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Connectable BLE advertising — role-independent single advertiser.
 *
 * Advertises whenever the tag can accept a connection (single-connection model,
 * BT_MAX_CONN=1):
 *   - disconnected + idle       -> ad_idle      (ESS only)      : phone services
 *   - disconnected + reflector  -> ad_reflector (RANGING + ESS) : CS initiator or phone
 *   - initiator (running)       -> off (device is central, needs its one slot)
 *   - connected                 -> off (slot in use); resumes on disconnect
 */
#include "adv.h"
#include "ble_core.h"
#include "cs_shared.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/ras.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_adv, CONFIG_LOG_DEFAULT_LEVEL);

/* Idle payload: Environmental Sensing UUID + name (phone-facing services). */
static const struct bt_data ad_idle[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Reflector payload: additionally advertise the Ranging Service UUID so a CS
 * initiator (RAS mode, scans by UUID) can find it. Harmless for IPT (name scan).
 */
static const struct bt_data ad_reflector[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

enum adv_variant {
	ADV_OFF,
	ADV_IDLE,
	ADV_REFLECTOR,
};

static K_MUTEX_DEFINE(adv_mtx);
static enum adv_variant current = ADV_OFF;

static enum adv_variant desired_variant(void)
{
	if (ble_peripheral_conn()) {
		return ADV_OFF; /* peripheral slot in use (screen/phone/SMP host) */
	}
	if (cs_is_running() && cs_get_role() == CS_ROLE_REFLECTOR) {
		return ADV_REFLECTOR; /* advertise Ranging UUID for the initiator */
	}
	/* Idle or initiator: advertise (ESS + name) so a screen/host can connect.
	 * As initiator we keep the separate outbound central link to the reflector.
	 */
	return ADV_IDLE;
}

void adv_refresh(void)
{
	k_mutex_lock(&adv_mtx, K_FOREVER);

	enum adv_variant want = desired_variant();

	if (want == current) {
		k_mutex_unlock(&adv_mtx);
		return;
	}

	if (current != ADV_OFF) {
		bt_le_adv_stop();
		current = ADV_OFF;
	}

	if (want != ADV_OFF) {
		const struct bt_data *ad =
			(want == ADV_REFLECTOR) ? ad_reflector : ad_idle;
		size_t len = (want == ADV_REFLECTOR) ? ARRAY_SIZE(ad_reflector)
						     : ARRAY_SIZE(ad_idle);
		int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, len, NULL, 0);

		if (err) {
			LOG_ERR("adv start failed (%d)", err);
		} else {
			current = want;
			LOG_INF("advertising: %s",
				want == ADV_REFLECTOR ? "reflector" : "idle");
		}
	} else {
		LOG_INF("advertising: off");
	}

	k_mutex_unlock(&adv_mtx);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(err);
	adv_refresh(); /* slot now in use (or connect failed) -> reconcile */
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(reason);
	adv_refresh(); /* slot free again -> resume per current role */
}

BT_CONN_CB_DEFINE(adv_conn_cbs) = {
	.connected = connected,
	.disconnected = disconnected,
};

int adv_init(void)
{
	adv_refresh();
	return 0;
}
