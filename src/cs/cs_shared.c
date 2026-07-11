/*
 * Channel Sounding shared core.
 *
 * Phase 0: state + lifecycle skeleton. Phase 3/4 fill in the CS handshake
 * (default settings, remote-caps, create_config, procedure params/enable)
 * and dispatch to the initiator/reflector modules.
 */
#include "cs_shared.h"
#include "cs_initiator.h"
#include "cs_reflector.h"

#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_cs, CONFIG_LOG_DEFAULT_LEVEL);

#define CS_DISTANCE_STALE_MS 2000

static float m_distance = NAN;
static int64_t m_distance_ts;

static enum cs_role m_role =
	IS_ENABLED(CONFIG_APP_CS_DEFAULT_ROLE_INITIATOR) ?
		CS_ROLE_INITIATOR : CS_ROLE_REFLECTOR;
static enum cs_mode m_mode =
	IS_ENABLED(CONFIG_APP_CS_DEFAULT_MODE_IPT) ?
		CS_MODE_IPT : CS_MODE_RAS;
static bool m_running;

const char *cs_role_str(enum cs_role role)
{
	return role == CS_ROLE_INITIATOR ? "initiator" : "reflector";
}

const char *cs_mode_str(enum cs_mode mode)
{
	return mode == CS_MODE_IPT ? "ipt" : "ras";
}

void cs_log_config(const struct bt_conn_le_cs_config *config)
{
	static const char *const mode_str[5] = {
		"Unused", "1 (RTT)", "2 (PBR)", "3 (RTT + PBR)", "Invalid"};
	static const char *const role_str[3] = {
		"Initiator", "Reflector", "Invalid"};
	static const char *const rtt_type_str[8] = {
		"AA only", "32-bit sounding", "96-bit sounding", "32-bit random",
		"64-bit random", "96-bit random", "128-bit random", "Invalid"};
	static const char *const phy_str[4] = {
		"Invalid", "LE 1M PHY", "LE 2M PHY", "LE 2M 2BT PHY"};
	static const char *const chsel_type_str[3] = {
		"Algorithm #3b", "Algorithm #3c", "Invalid"};
	static const char *const ch3c_shape_str[3] = {
		"Hat shape", "X shape", "Invalid"};

	uint8_t mode_idx = config->mode > 0 && config->mode < 4 ? config->mode : 4;
	uint8_t role_idx = MIN(config->role, 2);
	uint8_t rtt_type_idx = MIN(config->rtt_type, 7);
	uint8_t phy_idx = config->cs_sync_phy > 0 && config->cs_sync_phy < 4 ?
				  config->cs_sync_phy : 0;
	uint8_t chsel_type_idx = MIN(config->channel_selection_type, 2);
	uint8_t ch3c_shape_idx = MIN(config->ch3c_shape, 2);

	LOG_INF("CS config %u: mode=%s role=%s rtt=%s phy=%s chsel=%s ch3c=%s "
		"steps[%u..%u] rep=%u mode0=%u",
		config->id, mode_str[mode_idx], role_str[role_idx],
		rtt_type_str[rtt_type_idx], phy_str[phy_idx],
		chsel_type_str[chsel_type_idx], ch3c_shape_str[ch3c_shape_idx],
		config->min_main_mode_steps, config->max_main_mode_steps,
		config->main_mode_repetition, config->mode_0_steps);
}

int cs_init(void)
{
	LOG_INF("cs: init (role=%s mode=%s)", cs_role_str(m_role),
		cs_mode_str(m_mode));

	if (IS_ENABLED(CONFIG_APP_CS_AUTOSTART)) {
		return cs_start();
	}

	return 0;
}

int cs_set_role(enum cs_role role)
{
	if (m_running) {
		return -EBUSY;
	}
	m_role = role;
	return 0;
}

int cs_set_mode(enum cs_mode mode)
{
	if (m_running) {
		return -EBUSY;
	}
	m_mode = mode;
	return 0;
}

enum cs_role cs_get_role(void)
{
	return m_role;
}

enum cs_mode cs_get_mode(void)
{
	return m_mode;
}

bool cs_is_running(void)
{
	return m_running;
}

int cs_start(void)
{
	int err;

	if (m_running) {
		return -EALREADY;
	}

	LOG_INF("cs: start role=%s mode=%s", cs_role_str(m_role),
		cs_mode_str(m_mode));

	err = (m_role == CS_ROLE_INITIATOR) ?
		cs_initiator_start(m_mode) : cs_reflector_start(m_mode);
	if (err) {
		LOG_ERR("cs: start failed (%d)", err);
		return err;
	}

	m_running = true;
	return 0;
}

int cs_stop(void)
{
	if (!m_running) {
		return -EALREADY;
	}

	(m_role == CS_ROLE_INITIATOR) ? cs_initiator_stop() : cs_reflector_stop();
	m_running = false;
	m_distance = NAN;
	LOG_INF("cs: stopped");
	return 0;
}

void cs_report_distance(float meters)
{
	m_distance = meters;
	m_distance_ts = k_uptime_get();
}

bool cs_get_distance(float *meters_out)
{
	if (isnan(m_distance)) {
		return false;
	}
	if (k_uptime_get() - m_distance_ts > CS_DISTANCE_STALE_MS) {
		return false;
	}
	*meters_out = m_distance;
	return true;
}
