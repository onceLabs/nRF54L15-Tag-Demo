/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Channel Sounding reflector (peripheral) role.
 *
 * The reflector lets the initiator drive the CS procedure: it only sets its
 * default CS settings (reflector role) on connect and its procedure parameters
 * once the initiator has created the CS config. The initiator creates the
 * config (including the inline-PCT flag for IPT), so the reflector code is
 * identical for RAS and IPT. Advertising is owned by the ble adv module
 * (src/ble/adv.c), not here.
 *
 * CS event callbacks are registered here but gated on the active role so the
 * initiator's callbacks (which share the same events) do not double-handle.
 */
#include "cs_reflector.h"
#include "cs_shared.h"
#include "ble_core.h"

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app_cs, CONFIG_LOG_DEFAULT_LEVEL);

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_config, 0, 1);

static bool active;

static bool reflector_active(void)
{
	return active && cs_get_role() == CS_ROLE_REFLECTOR;
}

/* --- CS connection callbacks (gated on reflector role) --------------------- */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (!reflector_active() || err) {
		return;
	}
	k_sem_give(&sem_connected);
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	if (cs_get_role() != CS_ROLE_REFLECTOR) {
		return;
	}

	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("reflector: CS config create failed (0x%02x)", status);
		return;
	}

	cs_log_config(config);
	k_sem_give(&sem_config);
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	if (cs_get_role() != CS_ROLE_REFLECTOR) {
		return;
	}
	LOG_INF("reflector: CS security %s (0x%02x)",
		status == BT_HCI_ERR_SUCCESS ? "enabled" : "failed", status);
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (cs_get_role() != CS_ROLE_REFLECTOR) {
		return;
	}

	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("reflector: CS procedure enable failed (0x%02x)", status);
		return;
	}

	LOG_INF("reflector: CS procedures %s (config %u)",
		params->state == 1 ? "enabled" : "disabled", params->config_id);
}

BT_CONN_CB_DEFINE(cs_reflector_conn_cbs) = {
	.connected = connected_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
};

/* --- Reflector worker thread ---------------------------------------------- */

static void reflector_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		struct bt_conn *conn;
		int err;

		k_sem_take(&sem_connected, K_FOREVER);

		conn = ble_peripheral_conn();
		if (!conn) {
			continue;
		}

		/* The CS/RAS link must be encrypted (RAS chars are ENCRYPT-gated and
		 * CS requires an encrypted link). ble_core forces L2 only on central
		 * links, so as the peripheral we request it here. A plain screen/SMP
		 * host on an idle/initiator tag is left unencrypted (handled elsewhere).
		 */
		err = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (err) {
			LOG_WRN("reflector: set security L2 failed (%d)", err);
		}

		const struct bt_le_cs_set_default_settings_param default_settings = {
			.enable_initiator_role = false,
			.enable_reflector_role = true,
			.cs_sync_antenna_selection =
				BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
			.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
		};

		err = bt_le_cs_set_default_settings(conn, &default_settings);
		if (err) {
			LOG_ERR("reflector: default settings failed (%d)", err);
			continue;
		}

		/* Wait for the initiator to create the CS config. */
		k_sem_take(&sem_config, K_FOREVER);

		const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
			.config_id = 0,
			.max_procedure_len = 1000,
			.min_procedure_interval = 1,
			.max_procedure_interval = 100,
			.max_procedure_count = 0,
			.min_subevent_len = 10000,
			.max_subevent_len = 75000,
			.tone_antenna_config_selection =
				BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
			.phy = BT_LE_CS_PROCEDURE_PHY_2M,
			.tx_power_delta = 0x80,
			.preferred_peer_antenna =
				BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
			.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
			.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
		};

		err = bt_le_cs_set_procedure_parameters(conn, &procedure_params);
		if (err) {
			LOG_ERR("reflector: procedure params failed (%d)", err);
		}
	}
}

K_THREAD_DEFINE(cs_reflector_tid, 2048, reflector_thread, NULL, NULL, NULL,
		7, 0, 0);

/* --- Public API ----------------------------------------------------------- */

int cs_reflector_start(enum cs_mode mode)
{
	/* Advertising (incl. the Ranging UUID payload) is handled by the ble adv
	 * module, which cs_start() refreshes after this returns.
	 */
	active = true;
	LOG_INF("reflector: active (mode=%s)", cs_mode_str(mode));
	return 0;
}

void cs_reflector_stop(void)
{
	active = false;

	struct bt_conn *conn = ble_peripheral_conn();

	if (conn) {
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}
