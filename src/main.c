/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * nRF54L15 Tag Demo
 *
 * Dual-role, dual-mode (RAS + inline-PCT) Bluetooth Channel Sounding plus
 * onboard sensor sampling. Functional areas live in self-contained modules
 * under src/; each is enabled by its own CONFIG_APP_* symbol.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensors/sensors.h"
#include "ble/ble_core.h"
#include "ble/adv.h"
#include "cs/cs_shared.h"
#include "ess/ess.h"
#include "motion/motion.h"
#include "ux/ux.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
	int err;

	LOG_INF("nRF54L15 Tag Demo starting");

	err = sensors_init();
	if (err) {
		LOG_ERR("sensors_init failed (%d)", err);
	}

	err = ble_core_init();
	if (err) {
		LOG_ERR("ble_core_init failed (%d)", err);
	}

	err = adv_init();
	if (err) {
		LOG_ERR("adv_init failed (%d)", err);
	}

	err = cs_init();
	if (err) {
		LOG_ERR("cs_init failed (%d)", err);
	}

	err = ess_init();
	if (err) {
		LOG_ERR("ess_init failed (%d)", err);
	}

	err = motion_init();
	if (err) {
		LOG_ERR("motion_init failed (%d)", err);
	}

	err = ux_init();
	if (err) {
		LOG_ERR("ux_init failed (%d)", err);
	}

	LOG_INF("nRF54L15 Tag Demo ready");
	return 0;
}
