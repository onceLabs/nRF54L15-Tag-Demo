/*
 * Copyright (c) 2026 onceLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Distance-estimate post-processing: per-antenna-path sliding-window median
 * over the raw cs_de outputs. Ported from the ras_initiator reference.
 */
#ifndef APP_DISTANCE_H_
#define APP_DISTANCE_H_

#include <zephyr/kernel.h>
#include <bluetooth/cs_de.h>

/* Buffers are sized for the largest antenna-path count the build supports. */
#define DE_MAX_AP CONFIG_BT_CS_DE_MAX_NUM_ANTENNA_PATHS

/* Depth of the per-antenna-path sliding median window. */
#define DE_SLIDING_WINDOW_SIZE 9

/** @brief Clear all sliding windows (call on (re)connect). */
void distance_reset(void);

/** @brief Append one estimate for antenna path @p ap to its window. */
void distance_store(uint8_t ap, const cs_de_dist_estimates_t *estimates);

/** @brief Median (over finite samples) of antenna path @p ap's full window. */
cs_de_dist_estimates_t distance_get(uint8_t ap);

/** @brief Median over the most recent @p n samples of antenna path @p ap. */
cs_de_dist_estimates_t distance_get_recent(uint8_t ap, uint8_t n);

/** @brief Number of samples currently held for antenna path @p ap. */
uint8_t distance_count(uint8_t ap);

#endif /* APP_DISTANCE_H_ */
