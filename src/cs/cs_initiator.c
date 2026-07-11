/*
 * Channel Sounding initiator (central) role — RAS and inline-PCT (IPT).
 *
 * Ported and merged from the NCS channel_sounding ras_initiator and
 * ipt_initiator samples, with the transport selected at runtime:
 *
 *   RAS:  scan by Ranging Service UUID -> connect -> MTU exchange -> GATT
 *         discovery -> RREQ handles -> read features -> subscribe. CS config
 *         uses mode 2 sub-mode 1 (PBR+RTT). Local step data is collected in
 *         subevent_result_cb, peer step data is fetched over GATT, the two
 *         are paired and fed to cs_de_calc().
 *   IPT:  scan by device name -> connect. CS config uses mode 2 with the
 *         inline-PCT flag (cs_enhancements_1). Distance is computed locally
 *         from the initiator's own subevent IQ with cs_de_ifft() — no GATT.
 *
 * Both paths push results into the shared distance sliding-window module and
 * signal the print loop. The whole handshake runs in a dedicated thread; on
 * disconnect the session unwinds and scanning restarts (no reboot).
 */
#include "cs_initiator.h"
#include "cs_shared.h"
#include "distance.h"
#include "ble_core.h"
#include "sensors.h"

#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/cs_de.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app_cs, CONFIG_LOG_DEFAULT_LEVEL);

#define CS_CONFIG_ID			0
#define NUM_MODE_0_STEPS		3
#define CHANNEL_INDEX_OFFSET		2
#define TONE_QI_OK_TONE_COUNT_THRESHOLD 15
#define REFLECTOR_NAME			CONFIG_BT_DEVICE_NAME
#define HANDSHAKE_TIMEOUT		K_SECONDS(10)
#define PROCEDURE_COUNTER_NONE		(-1)

#define LOCAL_PROCEDURE_MEM \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) + \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_ras_features, 0, 1);
static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_local_steps, 1, 1);
static K_SEM_DEFINE(sem_distance_estimate_updated, 0, 1);

static bool active;
static enum cs_mode active_mode;

static struct bt_conn_le_cs_config cs_config;
static uint32_t ras_feature_bits;
static uint8_t last_n_ap = 1;

/* Scan params kept static so bt_scan can retain the conn_param pointer. */
static struct bt_le_conn_param scan_conn_param;
static struct bt_scan_init_param scan_params;
static bool scan_cb_registered;

/* --- RAS data path -------------------------------------------------------- */
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static uint16_t m_n_iqs[CONFIG_BT_CS_DE_MAX_NUM_ANTENNA_PATHS][CS_DE_NUM_CHANNELS];
static cs_de_report_t m_cs_de_report;

/* --- IPT data path -------------------------------------------------------- */
static union {
	struct {
		float i;
		float q;
	} values[CS_DE_NUM_CHANNELS];
	float scratch_mem[2 * CONFIG_BT_CS_DE_NFFT_SIZE];
} iq;
static struct k_work ipt_distance_work;

static bool initiator_active(void)
{
	return active && cs_get_role() == CS_ROLE_INITIATOR;
}

/* Store a new estimate, with accelerometer-assisted outlier rejection: while
 * the tag is stationary, drop estimates that jump implausibly far from the
 * current median (multipath spikes).
 */
static void submit_distance(uint8_t ap, const cs_de_dist_estimates_t *est)
{
#if IS_ENABLED(CONFIG_APP_CS_STABILIZE)
	if (sensors_is_stationary() && distance_count(ap) >= 3) {
		cs_de_dist_estimates_t cur =
			distance_get_recent(ap, DE_SLIDING_WINDOW_SIZE);
		float gate = CONFIG_APP_CS_STAB_GATE_MM / 1000.0f;

		if (isfinite(cur.ifft) && isfinite(est->ifft) &&
		    fabsf(est->ifft - cur.ifft) > gate) {
			return; /* reject spike while stationary */
		}
	}
#endif
	distance_store(ap, est);
}

/* ========================================================================= */
/* RAS: paired local/peer step parsing -> cs_de_calc                         */
/* ========================================================================= */

static void cumulate_mean(float *avg, float new_value, uint16_t *N)
{
	float a = 1.0f / (*N);
	float b = 1.0f - a;

	*avg = a * new_value + b * (*avg);
}

static bool m_is_tone_quality_ok(uint16_t num_iqs[CS_DE_NUM_CHANNELS],
				 uint8_t channel_map[10])
{
	uint8_t ok_tones = 0;

	for (uint8_t i = 0; i < CS_DE_NUM_CHANNELS; ++i) {
		if (BT_LE_CS_CHANNEL_BIT_GET(channel_map, i + CHANNEL_INDEX_OFFSET) &&
		    num_iqs[i] >= 1) {
			ok_tones++;
		}
	}
	return ok_tones >= TONE_QI_OK_TONE_COUNT_THRESHOLD;
}

static void extract_pcts(cs_de_report_t *p_report, uint8_t channel_index,
			 uint8_t antenna_permutation_index,
			 struct bt_hci_le_cs_step_data_tone_info *local_tone_info,
			 struct bt_hci_le_cs_step_data_tone_info *remote_tone_info)
{
	for (uint8_t tone_index = 0; tone_index < p_report->n_ap; tone_index++) {
		int antenna_path = bt_le_cs_get_antenna_path(
			p_report->n_ap, antenna_permutation_index, tone_index);

		if (antenna_path < 0) {
			LOG_WRN("Invalid antenna path.");
			return;
		}

		if (local_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH ||
		    remote_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
			return;
		}

		struct bt_le_cs_iq_sample local_iq = bt_le_cs_parse_pct(
			local_tone_info[tone_index].phase_correction_term);
		struct bt_le_cs_iq_sample remote_iq = bt_le_cs_parse_pct(
			remote_tone_info[tone_index].phase_correction_term);

		m_n_iqs[antenna_path][channel_index]++;

		if (m_n_iqs[antenna_path][channel_index] == 1) {
			p_report->iq_tones[antenna_path].i_local[channel_index] = local_iq.i;
			p_report->iq_tones[antenna_path].q_local[channel_index] = local_iq.q;
			p_report->iq_tones[antenna_path].i_remote[channel_index] = remote_iq.i;
			p_report->iq_tones[antenna_path].q_remote[channel_index] = remote_iq.q;
		} else {
			cumulate_mean(&p_report->iq_tones[antenna_path].i_local[channel_index],
				      local_iq.i, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].q_local[channel_index],
				      local_iq.q, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].i_remote[channel_index],
				      remote_iq.i, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].q_remote[channel_index],
				      remote_iq.q, &m_n_iqs[antenna_path][channel_index]);
		}
	}
}

static void extract_rtt_timings(cs_de_report_t *p_report,
				struct bt_hci_le_cs_step_data_mode_1 *local_rtt_data,
				struct bt_hci_le_cs_step_data_mode_1 *peer_rtt_data)
{
	if (local_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    local_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    local_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE ||
	    peer_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    peer_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    peer_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE) {
		return;
	}

	if (p_report->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
		p_report->rtt_accumulated_half_ns +=
			local_rtt_data->toa_tod_initiator - peer_rtt_data->tod_toa_reflector;
	} else {
		p_report->rtt_accumulated_half_ns +=
			peer_rtt_data->toa_tod_initiator - local_rtt_data->tod_toa_reflector;
	}

	p_report->rtt_count++;
}

static bool process_ranging_header(struct ras_ranging_header *ranging_header, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;

	p_report->n_ap = MAX(1, ((ranging_header->antenna_paths_mask & BIT(0)) +
				 ((ranging_header->antenna_paths_mask & BIT(1)) >> 1) +
				 ((ranging_header->antenna_paths_mask & BIT(2)) >> 2) +
				 ((ranging_header->antenna_paths_mask & BIT(3)) >> 3)));
	return true;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
			      struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;

	if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
		struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index,
			     local_step_data->tone_info, peer_step_data->tone_info);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1) {
		extract_rtt_timings(p_report,
				    (struct bt_hci_le_cs_step_data_mode_1 *)local_step->data,
				    (struct bt_hci_le_cs_step_data_mode_1 *)peer_step->data);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
		struct bt_hci_le_cs_step_data_mode_3 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_3 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index,
			     local_step_data->tone_info, peer_step_data->tone_info);
		extract_rtt_timings(p_report,
				    (struct bt_hci_le_cs_step_data_mode_1 *)local_step_data,
				    (struct bt_hci_le_cs_step_data_mode_1 *)peer_step_data);
	}

	return true;
}

static void ras_ranging_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	ARG_UNUSED(conn);

	if (err) {
		LOG_ERR("Ranging data error (counter %d err %d)", ranging_counter, err);
		return;
	}

	if (ranging_counter != most_recent_local_ranging_counter) {
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
		return;
	}

	if (latest_local_steps.len == 0) {
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
		if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
			net_buf_simple_reset(&latest_peer_steps);
		}
		return;
	}

	memset(&m_cs_de_report, 0, sizeof(m_cs_de_report));
	memset(m_n_iqs, 0, sizeof(m_n_iqs));

	bt_ras_rreq_rd_subevent_data_parse(&latest_peer_steps, &latest_local_steps,
					   cs_config.role, process_ranging_header, NULL,
					   process_step_data, &m_cs_de_report);

	for (uint8_t ap = 0; ap < m_cs_de_report.n_ap; ap++) {
		m_cs_de_report.distance_estimates[ap].ifft = NAN;
		m_cs_de_report.distance_estimates[ap].phase_slope = NAN;
		m_cs_de_report.distance_estimates[ap].rtt = NAN;
		m_cs_de_report.distance_estimates[ap].best = NAN;

		m_cs_de_report.tone_quality[ap] =
			m_is_tone_quality_ok(m_n_iqs[ap], cs_config.channel_map) ?
				CS_DE_TONE_QUALITY_OK : CS_DE_TONE_QUALITY_BAD;
	}

	net_buf_simple_reset(&latest_local_steps);
	if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
		net_buf_simple_reset(&latest_peer_steps);
	}
	k_sem_give(&sem_local_steps);

	if (cs_de_calc(&m_cs_de_report) != CS_DE_QUALITY_OK) {
		return;
	}

	last_n_ap = m_cs_de_report.n_ap;
	for (uint8_t ap = 0; ap < m_cs_de_report.n_ap; ap++) {
		if (m_cs_de_report.tone_quality[ap] == CS_DE_TONE_QUALITY_OK ||
		    isfinite(m_cs_de_report.distance_estimates[ap].rtt)) {
			submit_distance(ap, &m_cs_de_report.distance_estimates[ap]);
		}
	}
	k_sem_give(&sem_distance_estimate_updated);
}

static void ras_subevent_result(struct bt_conn *conn,
				struct bt_conn_le_cs_subevent_result *result)
{
	if (dropped_ranging_counter == result->header.procedure_counter) {
		return;
	}

	if (most_recent_local_ranging_counter !=
	    bt_ras_rreq_get_ranging_counter(result->header.procedure_counter)) {
		if (k_sem_take(&sem_local_steps, K_NO_WAIT) < 0) {
			dropped_ranging_counter = result->header.procedure_counter;
			return;
		}
		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	}

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		/* Steps from this subevent are unused. */
	} else if (result->step_data_buf) {
		if (result->step_data_buf->len <=
		    net_buf_simple_tailroom(&latest_local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			net_buf_simple_add_mem(&latest_local_steps, step_data, len);
		} else {
			LOG_ERR("Not enough memory for step data.");
			net_buf_simple_reset(&latest_local_steps);
			dropped_ranging_counter = result->header.procedure_counter;
			return;
		}
	}

	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
	}
}

static void ras_ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	if (ranging_counter != most_recent_local_ranging_counter) {
		return;
	}

	int err = bt_ras_rreq_cp_get_ranging_data(ble_current_conn(), &latest_peer_steps,
						  ranging_counter, ras_ranging_data_cb);
	if (err) {
		LOG_ERR("Get ranging data failed (err %d)", err);
		net_buf_simple_reset(&latest_local_steps);
		net_buf_simple_reset(&latest_peer_steps);
		k_sem_give(&sem_local_steps);
	}
}

static void ras_ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_INF("Ranging data overwritten %u", ranging_counter);
}

static void ras_features_read_cb(struct bt_conn *conn, uint32_t feature_bits, int err)
{
	if (err) {
		LOG_WRN("RAS feature read error (%d)", err);
	} else {
		ras_feature_bits = feature_bits;
	}
	k_sem_give(&sem_ras_features);
}

/* ========================================================================= */
/* IPT: local-only IQ -> cs_de_ifft                                          */
/* ========================================================================= */

static void ipt_pcts_parse(uint8_t channel_index,
			   struct bt_hci_le_cs_step_data_tone_info *local_tone_info)
{
	struct bt_le_cs_iq_sample local_iq =
		bt_le_cs_parse_pct(local_tone_info[0].phase_correction_term);

	iq.values[channel_index - CHANNEL_INDEX_OFFSET].i = local_iq.i;
	iq.values[channel_index - CHANNEL_INDEX_OFFSET].q = local_iq.q;
}

static void ipt_subevent_steps_parse(struct bt_conn_le_cs_subevent_result *result)
{
	for (uint8_t i = 0; i < result->header.num_steps_reported; i++) {
		if (result->step_data_buf->len < 3) {
			LOG_WRN("Local step data malformed.");
			return;
		}

		struct bt_le_cs_subevent_step local_step = {0};

		local_step.mode = net_buf_simple_pull_u8(result->step_data_buf);
		local_step.channel = net_buf_simple_pull_u8(result->step_data_buf);
		local_step.data_len = net_buf_simple_pull_u8(result->step_data_buf);

		if (local_step.data_len == 0 ||
		    local_step.data_len > result->step_data_buf->len) {
			LOG_WRN("Local step data malformed.");
			return;
		}

		local_step.data = result->step_data_buf->data;

		if (local_step.mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
			struct bt_hci_le_cs_step_data_mode_2 *sd =
				(struct bt_hci_le_cs_step_data_mode_2 *)local_step.data;
			ipt_pcts_parse(local_step.channel, sd->tone_info);
		}

		net_buf_simple_pull(result->step_data_buf, local_step.data_len);
	}
}

static void ipt_distance_work_handler(struct k_work *work)
{
	float distance_ifft = cs_de_ifft(iq.scratch_mem);

	if (isfinite(distance_ifft)) {
		cs_de_dist_estimates_t est = {
			.ifft = distance_ifft,
			.phase_slope = NAN,
			.rtt = NAN,
			.best = NAN,
		};

		last_n_ap = 1;
		submit_distance(0, &est);
		k_sem_give(&sem_distance_estimate_updated);
	}
}

static void ipt_subevent_result(struct bt_conn *conn,
				struct bt_conn_le_cs_subevent_result *result)
{
	static uint32_t prev_procedure_counter = UINT16_MAX + 1;
	static uint32_t dropped_procedure_counter = UINT16_MAX + 1;

	const bool cs_aborted =
		(result->header.procedure_abort_reason !=
		 BT_HCI_LE_CS_PROCEDURE_ABORT_REASON_NO_ABORT) ||
		(result->header.subevent_abort_reason !=
		 BT_HCI_LE_CS_SUBEVENT_ABORT_REASON_NO_ABORT);

	if (cs_aborted) {
		return;
	}

	if (result->header.procedure_counter == dropped_procedure_counter) {
		return;
	}

	if (k_work_is_pending(&ipt_distance_work)) {
		dropped_procedure_counter = result->header.procedure_counter;
		return;
	}

	if (result->header.procedure_counter != prev_procedure_counter) {
		memset(iq.scratch_mem, 0, sizeof(iq.scratch_mem));
	}
	prev_procedure_counter = result->header.procedure_counter;

	ipt_subevent_steps_parse(result);

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		k_work_submit(&ipt_distance_work);
	}
}

/* ========================================================================= */
/* Shared CS + connection callbacks (gated on initiator role)                */
/* ========================================================================= */

static void subevent_result_cb(struct bt_conn *conn,
			       struct bt_conn_le_cs_subevent_result *result)
{
	if (!initiator_active()) {
		return;
	}

	if (active_mode == CS_MODE_RAS) {
		ras_subevent_result(conn, result);
	} else {
		ipt_subevent_result(conn, result);
	}
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (!initiator_active() || err) {
		return;
	}
	k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (cs_get_role() != CS_ROLE_INITIATOR) {
		return;
	}
	/* Unblock the session thread; it will re-scan if still active. */
	k_sem_give(&sem_connected);
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err err)
{
	if (cs_get_role() != CS_ROLE_INITIATOR || err) {
		return;
	}
	k_sem_give(&sem_security);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_ERR("MTU exchange failed (err %d)", err);
		return;
	}
	k_sem_give(&sem_mtu_exchange_done);
}

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	if (cs_get_role() != CS_ROLE_INITIATOR) {
		return;
	}
	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("initiator: CS capability exchange complete");
		k_sem_give(&sem_remote_capabilities_obtained);
	} else {
		LOG_WRN("initiator: CS capability exchange failed (0x%02x)", status);
	}
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	if (cs_get_role() != CS_ROLE_INITIATOR) {
		return;
	}
	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("initiator: CS config create failed (0x%02x)", status);
		return;
	}
	cs_config = *config;
	cs_log_config(config);
	k_sem_give(&sem_config_created);
}

static void cs_security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	if (cs_get_role() != CS_ROLE_INITIATOR) {
		return;
	}
	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_INF("initiator: CS security enabled");
		k_sem_give(&sem_cs_security_enabled);
	} else {
		LOG_WRN("initiator: CS security enable failed (0x%02x)", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (cs_get_role() != CS_ROLE_INITIATOR) {
		return;
	}
	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("initiator: CS procedure enable failed (0x%02x)", status);
		return;
	}
	LOG_INF("initiator: CS procedures %s (config %u)",
		params->state == 1 ? "enabled" : "disabled", params->config_id);
}

BT_CONN_CB_DEFINE(cs_initiator_conn_cbs) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = cs_security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

/* --- GATT discovery (RAS only) -------------------------------------------- */

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err = bt_ras_rreq_alloc_and_assign_handles(dm, bt_gatt_dm_conn_get(dm));

	if (err) {
		LOG_ERR("RAS RREQ alloc failed (err %d)", err);
	}

	bt_gatt_dm_data_release(dm);
	k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_WRN("Ranging Service not found; disconnecting");
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_WRN("Discovery failed (err %d)", err);
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

/* --- Scanning ------------------------------------------------------------- */

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
	LOG_INF("Scan match: %s connectable %d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Connect failed, restarting scan");
	bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, NULL);

static int scan_setup(enum cs_mode mode, uint16_t conn_interval)
{
	int err;

	scan_conn_param.interval_min = conn_interval;
	scan_conn_param.interval_max = conn_interval;
	scan_conn_param.latency = 0;
	scan_conn_param.timeout = BT_GAP_MS_TO_CONN_TIMEOUT(4000);

	scan_params.scan_param = NULL;
	scan_params.conn_param = &scan_conn_param;
	scan_params.connect_if_match = 1;

	bt_scan_init(&scan_params);
	if (!scan_cb_registered) {
		bt_scan_cb_register(&scan_cb);
		scan_cb_registered = true;
	}

	bt_scan_filter_remove_all();

	if (mode == CS_MODE_RAS) {
		err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
		if (err) {
			LOG_ERR("UUID scan filter add failed (err %d)", err);
			return err;
		}
		err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	} else {
		err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, REFLECTOR_NAME);
		if (err) {
			LOG_ERR("Name scan filter add failed (err %d)", err);
			return err;
		}
		err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
	}

	if (err) {
		LOG_ERR("Scan filter enable failed (err %d)", err);
	}
	return err;
}

/* --- Handshake sequence --------------------------------------------------- */

static void cs_config_get(struct bt_le_cs_create_config_params *p, enum cs_mode mode)
{
	memset(p, 0, sizeof(*p));
	p->id = CS_CONFIG_ID;
	p->mode = (mode == CS_MODE_RAS) ? BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1
					: BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE;
	p->min_main_mode_steps = 2;
	p->max_main_mode_steps = 5;
	p->main_mode_repetition = 0;
	p->mode_0_steps = NUM_MODE_0_STEPS;
	p->role = BT_CONN_LE_CS_ROLE_INITIATOR;
	p->rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;
	p->cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY;
	p->channel_map_repetition = 1;
	p->channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	p->ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT;
	p->ch3c_jump = 2;
	if (mode == CS_MODE_IPT) {
		p->cs_enhancements_1 = 1; /* inline PCT transfer */
	}
}

static int ras_setup(struct bt_conn *conn, uint16_t conn_interval)
{
	int err;
	static struct bt_gatt_exchange_params mtu_params = {.func = mtu_exchange_cb};

	bt_gatt_exchange_mtu(conn, &mtu_params);
	if (k_sem_take(&sem_mtu_exchange_done, HANDSHAKE_TIMEOUT)) {
		LOG_ERR("initiator: MTU exchange timeout");
		return -ETIMEDOUT;
	}

	err = bt_gatt_dm_start(conn, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
	if (err) {
		LOG_ERR("initiator: discovery start failed (%d)", err);
		return err;
	}
	if (k_sem_take(&sem_discovery_done, HANDSHAKE_TIMEOUT)) {
		LOG_ERR("initiator: discovery timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ras_subscribe(struct bt_conn *conn, bool *realtime_out)
{
	int err = bt_ras_rreq_read_features(conn, ras_features_read_cb);

	if (err) {
		LOG_ERR("initiator: read RAS features failed (%d)", err);
		return err;
	}
	if (k_sem_take(&sem_ras_features, HANDSHAKE_TIMEOUT)) {
		return -ETIMEDOUT;
	}

	bool realtime = ras_feature_bits & RAS_FEAT_REALTIME_RD;
	*realtime_out = realtime;

	if (realtime) {
		err = bt_ras_rreq_realtime_rd_subscribe(conn, &latest_peer_steps,
							ras_ranging_data_cb);
		if (err) {
			LOG_ERR("initiator: realtime RD subscribe failed (%d)", err);
			return err;
		}
	} else {
		err = bt_ras_rreq_rd_overwritten_subscribe(conn,
							   ras_ranging_data_overwritten_cb);
		if (!err) {
			err = bt_ras_rreq_rd_ready_subscribe(conn, ras_ranging_data_ready_cb);
		}
		if (!err) {
			err = bt_ras_rreq_on_demand_rd_subscribe(conn);
		}
		if (!err) {
			err = bt_ras_rreq_cp_subscribe(conn);
		}
		if (err) {
			LOG_ERR("initiator: RAS subscribe failed (%d)", err);
			return err;
		}
	}

	return 0;
}

static int run_session(struct bt_conn *conn, enum cs_mode mode, uint16_t conn_interval)
{
	int err;
	bool realtime = false;

	if (k_sem_take(&sem_security, HANDSHAKE_TIMEOUT)) {
		LOG_ERR("initiator: security timeout");
		return -ETIMEDOUT;
	}

	if (mode == CS_MODE_RAS) {
		err = ras_setup(conn, conn_interval);
		if (err) {
			return err;
		}
	}

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(conn, &default_settings);
	if (err) {
		LOG_ERR("initiator: default settings failed (%d)", err);
		return err;
	}

	if (mode == CS_MODE_RAS) {
		err = ras_subscribe(conn, &realtime);
		if (err) {
			return err;
		}
	}

	err = bt_le_cs_read_remote_supported_capabilities(conn);
	if (err) {
		LOG_ERR("initiator: read remote caps failed (%d)", err);
		return err;
	}
	if (k_sem_take(&sem_remote_capabilities_obtained, HANDSHAKE_TIMEOUT)) {
		return -ETIMEDOUT;
	}

	struct bt_le_cs_create_config_params config_params;

	cs_config_get(&config_params, mode);
	bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

	err = bt_le_cs_create_config(conn, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("initiator: create config failed (%d)", err);
		return err;
	}
	if (k_sem_take(&sem_config_created, HANDSHAKE_TIMEOUT)) {
		return -ETIMEDOUT;
	}

	err = bt_le_cs_security_enable(conn);
	if (err) {
		LOG_ERR("initiator: CS security enable failed (%d)", err);
		return err;
	}
	if (k_sem_take(&sem_cs_security_enabled, HANDSHAKE_TIMEOUT)) {
		return -ETIMEDOUT;
	}

	const uint16_t acl_units = conn_interval * 2;
	uint16_t proc_interval = (mode == CS_MODE_RAS) ? (realtime ? 5 : 10) : 2;
	uint16_t max_proc_len = acl_units * (proc_interval - 1);
	uint16_t subevent_len = (mode == CS_MODE_RAS) ? 16000 : 11000;

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = max_proc_len,
		.min_procedure_interval = proc_interval,
		.max_procedure_interval = proc_interval,
		.max_procedure_count = 0,
		.min_subevent_len = subevent_len,
		.max_subevent_len = subevent_len,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
		.phy = BT_LE_CS_PROCEDURE_PHY_2M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(conn, &procedure_params);
	if (err) {
		LOG_ERR("initiator: set procedure params failed (%d)", err);
		return err;
	}

	struct bt_le_cs_procedure_enable_param en = {
		.config_id = CS_CONFIG_ID,
		.enable = 1,
	};

	err = bt_le_cs_procedure_enable(conn, &en);
	if (err) {
		LOG_ERR("initiator: procedure enable failed (%d)", err);
		return err;
	}

	return 0;
}

/* Apply the static calibration offset (RF/antenna path bias), clamped >= 0. */
static float apply_offset(float m)
{
	if (!isfinite(m)) {
		return m; /* preserve NaN */
	}

	float v = m + (CONFIG_APP_CS_DISTANCE_OFFSET_MM / 1000.0f);

	return v < 0.0f ? 0.0f : v;
}

static void print_distances(enum cs_mode mode)
{
	uint8_t win = DE_SLIDING_WINDOW_SIZE;

#if IS_ENABLED(CONFIG_APP_CS_STABILIZE)
	/* Stationary: full window (steady). Moving: short window (responsive). */
	win = sensors_is_stationary() ? DE_SLIDING_WINDOW_SIZE
				      : CONFIG_APP_CS_STAB_WINDOW_MOVING;
#endif

	for (uint8_t ap = 0; ap < last_n_ap && ap < DE_MAX_AP; ap++) {
		if (distance_count(ap) == 0) {
			continue;
		}

		cs_de_dist_estimates_t d = distance_get_recent(ap, win);
		float ifft = apply_offset(d.ifft);
		float phase_slope = apply_offset(d.phase_slope);
		float rtt = apply_offset(d.rtt);

		if (mode == CS_MODE_IPT) {
			LOG_INF("distance[ap%u]: ifft %.2f m", ap, (double)ifft);
		} else {
			LOG_INF("distance[ap%u]: ifft %.2f  phase_slope %.2f  rtt %.2f m",
				ap, (double)ifft, (double)phase_slope, (double)rtt);
		}

		/* Publish antenna path 0's ifft distance for the LED UX. */
		if (ap == 0 && isfinite(ifft)) {
			cs_report_distance(ifft);
		}
	}
}

static void reset_session_state(void)
{
	net_buf_simple_reset(&latest_local_steps);
	net_buf_simple_reset(&latest_peer_steps);
	most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
	ras_feature_bits = 0;
	k_sem_reset(&sem_local_steps);
	k_sem_give(&sem_local_steps);
	distance_reset();
}

static void initiator_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_work_init(&ipt_distance_work, ipt_distance_work_handler);

	while (1) {
		if (!initiator_active()) {
			k_sleep(K_MSEC(200));
			continue;
		}

		/* Wait for a connection (given by connected_cb). */
		if (k_sem_take(&sem_connected, K_SECONDS(30))) {
			continue; /* re-check active / keep scanning */
		}

		struct bt_conn *conn = ble_current_conn();

		if (!conn) {
			continue; /* disconnected wake-up */
		}

		reset_session_state();

		uint16_t conn_interval = (active_mode == CS_MODE_RAS) ? 16 : 12;
		int err = run_session(conn, active_mode, conn_interval);

		if (err) {
			LOG_WRN("initiator: session setup failed (%d)", err);
			if (ble_current_conn()) {
				bt_conn_disconnect(conn,
						   BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			}
			continue;
		}

		LOG_INF("initiator: ranging (mode=%s)", cs_mode_str(active_mode));

		/* Report until disconnect or stop. */
		while (initiator_active() && ble_current_conn()) {
			if (k_sem_take(&sem_distance_estimate_updated, K_MSEC(500)) == 0) {
				print_distances(active_mode);
			}
		}

		/* Session ended (disconnect). Resume scanning if still active
		 * — bt_scan auto-connect stops the scanner on connection.
		 */
		if (initiator_active() && !ble_current_conn()) {
			int serr = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);

			if (serr && serr != -EALREADY) {
				LOG_WRN("initiator: scan restart failed (%d)", serr);
			}
		}
	}
}

K_THREAD_DEFINE(cs_initiator_tid, 4096, initiator_thread, NULL, NULL, NULL,
		7, 0, 0);

/* --- Public API ----------------------------------------------------------- */

int cs_initiator_start(enum cs_mode mode)
{
	int err;

	active_mode = mode;
	active = true;

	err = scan_setup(mode, (mode == CS_MODE_RAS) ? 16 : 12);
	if (err) {
		active = false;
		return err;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("initiator: scan start failed (%d)", err);
		active = false;
		return err;
	}

	LOG_INF("initiator: scanning (mode=%s)", cs_mode_str(mode));
	return 0;
}

void cs_initiator_stop(void)
{
	active = false;
	bt_scan_stop();

	struct bt_conn *conn = ble_current_conn();

	if (conn) {
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}
