/*
 * Button + RGB-LED UX for the two-tag Channel Sounding demo.
 */
#ifndef APP_UX_H_
#define APP_UX_H_

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_APP_UX)

/**
 * @brief Initialise the button (role cycling) and the RGB LED indicator.
 *
 * @return 0 on success, negative errno otherwise.
 */
int ux_init(void);

#else

static inline int ux_init(void)
{
	return 0;
}

#endif /* CONFIG_APP_UX */

#endif /* APP_UX_H_ */
