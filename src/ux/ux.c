/*
 * Button + RGB-LED UX for the two-tag Channel Sounding demo.
 *
 * Button (sw0) cycles the CS role: Off -> Reflector -> Initiator -> Off.
 * The RGB LED (led1_red/green/blue) shows the state:
 *   Off        -> LED off
 *   Reflector  -> solid blue
 *   Initiator  -> solid white while searching/connecting; once ranging, the
 *                 zone color (green<near, yellow<far, red beyond) blinks at a
 *                 distance-derived rate (closer = faster).
 *
 * This module owns the RGB LED (ble_core relinquishes it when APP_UX is set).
 */
#include "ux.h"
#include "cs_shared.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(app_ux, CONFIG_LOG_DEFAULT_LEVEL);

enum ux_role {
	UX_OFF,
	UX_REFLECTOR,
	UX_INITIATOR,
};

static atomic_t m_role = ATOMIC_INIT(UX_OFF);

static const struct gpio_dt_spec led_r =
	GPIO_DT_SPEC_GET(DT_NODELABEL(led1_red), gpios);
static const struct gpio_dt_spec led_g =
	GPIO_DT_SPEC_GET(DT_NODELABEL(led1_green), gpios);
static const struct gpio_dt_spec led_b =
	GPIO_DT_SPEC_GET(DT_NODELABEL(led1_blue), gpios);

static void set_rgb(bool r, bool g, bool b)
{
	gpio_pin_set_dt(&led_r, r);
	gpio_pin_set_dt(&led_g, g);
	gpio_pin_set_dt(&led_b, b);
}

/* --- Role cycling (button) ------------------------------------------------ */

static void apply_role(enum ux_role role)
{
	atomic_set(&m_role, role);

	cs_stop(); /* harmless if not running; required before cs_set_role */

	switch (role) {
	case UX_REFLECTOR:
		cs_set_role(CS_ROLE_REFLECTOR);
		cs_start();
		LOG_INF("ux: role -> reflector");
		break;
	case UX_INITIATOR:
		cs_set_role(CS_ROLE_INITIATOR);
		cs_start();
		LOG_INF("ux: role -> initiator");
		break;
	default:
		LOG_INF("ux: role -> off");
		break;
	}
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if (!(has_changed & DK_BTN1_MSK) || !(button_state & DK_BTN1_MSK)) {
		return; /* only act on press (rising edge) of button 1 */
	}

	enum ux_role cur = (enum ux_role)atomic_get(&m_role);
	enum ux_role next = (cur == UX_OFF)       ? UX_REFLECTOR :
			    (cur == UX_REFLECTOR) ? UX_INITIATOR :
						    UX_OFF;

	apply_role(next);
}

/* --- LED indicator thread ------------------------------------------------- */

static void distance_color(float m, bool *r, bool *g, bool *b)
{
	float near_m = CONFIG_APP_UX_NEAR_MM / 1000.0f;
	float far_m = CONFIG_APP_UX_FAR_MM / 1000.0f;

	if (m < near_m) {
		*r = false; *g = true; *b = false;   /* green */
	} else if (m < far_m) {
		*r = true; *g = true; *b = false;    /* yellow */
	} else {
		*r = true; *g = false; *b = false;   /* red */
	}
}

static uint32_t distance_period_ms(float m)
{
	int32_t p = (int32_t)(200.0f + m * 300.0f); /* closer = faster */

	return (uint32_t)CLAMP(p, 200, 2000);
}

static void led_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		enum ux_role role = (enum ux_role)atomic_get(&m_role);
		float m;

		switch (role) {
		case UX_OFF:
			set_rgb(false, false, false);
			k_sleep(K_MSEC(100));
			break;
		case UX_REFLECTOR:
			set_rgb(false, false, true); /* solid blue */
			k_sleep(K_MSEC(100));
			break;
		case UX_INITIATOR:
			if (cs_get_distance(&m)) {
				bool r, g, b;
				uint32_t period = distance_period_ms(m);

				distance_color(m, &r, &g, &b);
				set_rgb(r, g, b);
				k_sleep(K_MSEC(period / 2));
				set_rgb(false, false, false);
				k_sleep(K_MSEC(period / 2));
			} else {
				set_rgb(true, true, true); /* white: searching */
				k_sleep(K_MSEC(150));
			}
			break;
		}
	}
}

K_THREAD_DEFINE(ux_led_tid, 1024, led_thread, NULL, NULL, NULL, 7, 0, 0);

int ux_init(void)
{
	if (!gpio_is_ready_dt(&led_r) || !gpio_is_ready_dt(&led_g) ||
	    !gpio_is_ready_dt(&led_b)) {
		LOG_ERR("ux: RGB LED not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);

	int err = dk_buttons_init(button_handler);

	if (err) {
		LOG_ERR("ux: dk_buttons_init failed (%d)", err);
		return err;
	}

	LOG_INF("ux: ready (button cycles off/reflector/initiator)");
	return 0;
}
