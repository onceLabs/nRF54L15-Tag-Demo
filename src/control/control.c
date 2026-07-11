/*
 * Runtime control shell.
 *
 *   cs role <initiator|reflector>
 *   cs mode <ras|ipt>
 *   cs start | stop | status
 *
 *   sensor log <env|accel|imu|all> <on|off>   (per-peripheral log gate)
 *   sensor status
 *
 * Whole-module logs (app_ble_core, app_cs, app_sensors, main) are toggled
 * with Zephyr's built-in `log enable/disable <level> <module>` command,
 * available because CONFIG_LOG_RUNTIME_FILTERING=y.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "cs_shared.h"
#include "sensors.h"

#if IS_ENABLED(CONFIG_APP_CS)

static int cmd_cs_role(const struct shell *sh, size_t argc, char **argv)
{
	enum cs_role role;
	int err;

	if (!strcmp(argv[1], "initiator") || !strcmp(argv[1], "init")) {
		role = CS_ROLE_INITIATOR;
	} else if (!strcmp(argv[1], "reflector") || !strcmp(argv[1], "refl")) {
		role = CS_ROLE_REFLECTOR;
	} else {
		shell_error(sh, "role must be initiator|reflector");
		return -EINVAL;
	}

	err = cs_set_role(role);
	if (err) {
		shell_error(sh, "cannot change role while running (%d)", err);
		return err;
	}

	shell_print(sh, "role = %s", cs_role_str(role));
	return 0;
}

static int cmd_cs_mode(const struct shell *sh, size_t argc, char **argv)
{
	enum cs_mode mode;
	int err;

	if (!strcmp(argv[1], "ras")) {
		mode = CS_MODE_RAS;
	} else if (!strcmp(argv[1], "ipt")) {
		mode = CS_MODE_IPT;
	} else {
		shell_error(sh, "mode must be ras|ipt");
		return -EINVAL;
	}

	err = cs_set_mode(mode);
	if (err) {
		shell_error(sh, "cannot change mode while running (%d)", err);
		return err;
	}

	shell_print(sh, "mode = %s", cs_mode_str(mode));
	return 0;
}

static int cmd_cs_start(const struct shell *sh, size_t argc, char **argv)
{
	int err = cs_start();

	if (err) {
		shell_error(sh, "start failed (%d)", err);
		return err;
	}
	return 0;
}

static int cmd_cs_stop(const struct shell *sh, size_t argc, char **argv)
{
	int err = cs_stop();

	if (err) {
		shell_error(sh, "stop failed (%d)", err);
		return err;
	}
	return 0;
}

static int cmd_cs_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "role=%s mode=%s running=%s",
		    cs_role_str(cs_get_role()), cs_mode_str(cs_get_mode()),
		    cs_is_running() ? "yes" : "no");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(cs_cmds,
	SHELL_CMD_ARG(role, NULL, "Set role: initiator|reflector",
		      cmd_cs_role, 2, 0),
	SHELL_CMD_ARG(mode, NULL, "Set mode: ras|ipt", cmd_cs_mode, 2, 0),
	SHELL_CMD_ARG(start, NULL, "Start ranging", cmd_cs_start, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop ranging", cmd_cs_stop, 1, 0),
	SHELL_CMD_ARG(status, NULL, "Show role/mode/running", cmd_cs_status, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cs, &cs_cmds, "Channel Sounding control", NULL);

#endif /* CONFIG_APP_CS */

#if IS_ENABLED(CONFIG_APP_SENSORS)

static int parse_on_off(const struct shell *sh, const char *arg, bool *enable)
{
	if (!strcmp(arg, "on")) {
		*enable = true;
	} else if (!strcmp(arg, "off")) {
		*enable = false;
	} else {
		shell_error(sh, "expected on|off");
		return -EINVAL;
	}
	return 0;
}

static int cmd_sensor_log(const struct shell *sh, size_t argc, char **argv)
{
	bool enable;

	if (parse_on_off(sh, argv[2], &enable)) {
		return -EINVAL;
	}

	if (!strcmp(argv[1], "all")) {
		for (enum sensor_id id = 0; id < SENSOR_ID_COUNT; id++) {
			sensors_set_log(id, enable);
		}
		shell_print(sh, "all sensor logs %s", enable ? "on" : "off");
		return 0;
	}

	for (enum sensor_id id = 0; id < SENSOR_ID_COUNT; id++) {
		if (!strcmp(argv[1], sensors_id_name(id))) {
			sensors_set_log(id, enable);
			shell_print(sh, "%s log %s", sensors_id_name(id),
				    enable ? "on" : "off");
			return 0;
		}
	}

	shell_error(sh, "unknown peripheral '%s' (use env|accel|imu|all)", argv[1]);
	return -EINVAL;
}

static int parse_motion_chan(const struct shell *sh, const char *arg,
			     enum motion_chan *ch)
{
	for (enum motion_chan c = 0; c < MOTION_CHAN_COUNT; c++) {
		if (!strcmp(arg, sensors_motion_name(c))) {
			*ch = c;
			return 0;
		}
	}
	shell_error(sh, "unknown channel '%s' (use accel|imu-accel|imu-gyro)", arg);
	return -EINVAL;
}

static int cmd_sensor_odr(const struct shell *sh, size_t argc, char **argv)
{
	enum motion_chan ch;

	if (parse_motion_chan(sh, argv[1], &ch)) {
		return -EINVAL;
	}

	uint32_t hz = strtoul(argv[2], NULL, 10);
	int err = sensors_set_odr(ch, hz);

	if (err == -EINVAL) {
		if (ch == MOTION_ACCEL) {
			shell_error(sh, "accel ODR must be 12|25|50|100|200|400 Hz");
		} else if (ch == MOTION_IMU_ACCEL) {
			shell_error(sh, "imu-accel ODR must be 1..1600 Hz");
		} else {
			shell_error(sh, "imu-gyro ODR must be 25..3200 Hz");
		}
		return err;
	}
	if (err) {
		shell_error(sh, "set ODR failed (%d)", err);
		return err;
	}

	shell_print(sh, "%s ODR = %u Hz", sensors_motion_name(ch),
		    sensors_get_odr(ch));
	return 0;
}

static int cmd_sensor_range(const struct shell *sh, size_t argc, char **argv)
{
	enum motion_chan ch;

	if (parse_motion_chan(sh, argv[1], &ch)) {
		return -EINVAL;
	}

	uint32_t range = strtoul(argv[2], NULL, 10);
	int err = sensors_set_range(ch, range);

	if (err == -ENOTSUP) {
		shell_error(sh, "ADXL367 range is devicetree-fixed (default +/-2G)");
		return err;
	}
	if (err == -EINVAL) {
		if (ch == MOTION_IMU_ACCEL) {
			shell_error(sh, "imu-accel range must be 2|4|8|16 G");
		} else {
			shell_error(sh, "imu-gyro range must be 125|250|500|1000|2000 dps");
		}
		return err;
	}
	if (err) {
		shell_error(sh, "set range failed (%d)", err);
		return err;
	}

	shell_print(sh, "%s range = %u %s", sensors_motion_name(ch),
		    sensors_get_range(ch), ch == MOTION_IMU_GYRO ? "dps" : "G");
	return 0;
}

static int cmd_sensor_stream(const struct shell *sh, size_t argc, char **argv)
{
	if (!strcmp(argv[1], "start")) {
		int err = sensors_stream_start();

		if (err) {
			shell_error(sh, "stream start failed (%d)", err);
			return err;
		}
		shell_print(sh, "stream started");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		sensors_stream_stop();
		shell_print(sh, "stream stopped");
		return 0;
	}

	if (!strcmp(argv[1], "rate")) {
		if (argc < 3) {
			shell_error(sh, "usage: sensor stream rate <hz> (0=track ODR)");
			return -EINVAL;
		}
		uint32_t hz = strtoul(argv[2], NULL, 10);

		sensors_stream_set_rate(hz);
		if (hz == 0) {
			shell_print(sh, "stream rate = ODR (tracking)");
		} else {
			shell_print(sh, "stream rate = %u Hz", hz);
		}
		return 0;
	}

	shell_error(sh, "usage: sensor stream <start|stop|rate <hz>>");
	return -EINVAL;
}

static int cmd_sensor_status(const struct shell *sh, size_t argc, char **argv)
{
	for (enum sensor_id id = 0; id < SENSOR_ID_COUNT; id++) {
		shell_print(sh, "%-6s log %s", sensors_id_name(id),
			    sensors_log_enabled(id) ? "on" : "off");
	}

	for (enum motion_chan ch = 0; ch < MOTION_CHAN_COUNT; ch++) {
		const char *unit = (ch == MOTION_IMU_GYRO) ? "dps" : "G";
		const char *fixed = (ch == MOTION_ACCEL) ? " (fixed)" : "";

		shell_print(sh, "%-9s ODR %u Hz  range %u %s%s",
			    sensors_motion_name(ch), sensors_get_odr(ch),
			    sensors_get_range(ch), unit, fixed);
	}

	uint32_t poll = sensors_stream_get_rate();
	char poll_str[16];

	if (poll) {
		snprintf(poll_str, sizeof(poll_str), "%u Hz", poll);
	} else {
		strcpy(poll_str, "ODR");
	}

	shell_print(sh, "stream    %s  poll %s  measured %u Hz  dropped %u",
		    sensors_stream_active() ? "active" : "idle", poll_str,
		    sensors_stream_rate_hz(), sensors_stream_dropped());
	shell_print(sh, "motion    %s",
		    sensors_is_stationary() ? "stationary" : "moving");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sensor_cmds,
	SHELL_CMD_ARG(log, NULL, "Gate logs: <env|accel|imu|all> <on|off>",
		      cmd_sensor_log, 3, 0),
	SHELL_CMD_ARG(odr, NULL, "Set ODR (Hz): <accel|imu-accel|imu-gyro> <hz>",
		      cmd_sensor_odr, 3, 0),
	SHELL_CMD_ARG(range, NULL,
		      "Set full-scale: <accel|imu-accel|imu-gyro> <G|dps>",
		      cmd_sensor_range, 3, 0),
	SHELL_CMD_ARG(stream, NULL,
		      "IMU-accel stream: start | stop | rate <hz>",
		      cmd_sensor_stream, 2, 1),
	SHELL_CMD_ARG(status, NULL, "Show log state, ODR, range and stream",
		      cmd_sensor_status, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sensor, &sensor_cmds, "Onboard sensor control", NULL);

#endif /* CONFIG_APP_SENSORS */
