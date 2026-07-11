/*
 * Channel Sounding shared core: role/mode state and lifecycle.
 *
 * Both roles (initiator/reflector) and both transports (RAS/IPT) are
 * selectable at runtime; only one combination is active per session and
 * the peer must match.
 */
#ifndef APP_CS_SHARED_H_
#define APP_CS_SHARED_H_

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/cs.h>

enum cs_role {
	CS_ROLE_INITIATOR,   /* central, drives CS and computes distance */
	CS_ROLE_REFLECTOR,   /* peripheral, being located */
};

enum cs_mode {
	CS_MODE_RAS,         /* GATT Ranging Service transfer */
	CS_MODE_IPT,         /* inline PCT, local estimation  */
};

const char *cs_role_str(enum cs_role role);
const char *cs_mode_str(enum cs_mode mode);

/** @brief Pretty-print a completed CS configuration to the log. */
void cs_log_config(const struct bt_conn_le_cs_config *config);

#if IS_ENABLED(CONFIG_APP_CS)

/** @brief Initialise the CS subsystem with the Kconfig boot defaults. */
int cs_init(void);

/** @brief Select the CS role. Only takes effect on the next cs_start(). */
int cs_set_role(enum cs_role role);

/** @brief Select the ranging mode. Only takes effect on the next cs_start(). */
int cs_set_mode(enum cs_mode mode);

enum cs_role cs_get_role(void);
enum cs_mode cs_get_mode(void);
bool cs_is_running(void);

/** @brief Start scanning/advertising for the selected role/mode. */
int cs_start(void);

/** @brief Stop any active CS activity and disconnect. */
int cs_stop(void);

#else

static inline int cs_init(void)
{
	return 0;
}

#endif /* CONFIG_APP_CS */

#endif /* APP_CS_SHARED_H_ */
