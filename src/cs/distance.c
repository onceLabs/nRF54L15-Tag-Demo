/*
 * Distance-estimate post-processing: per-antenna-path sliding-window median.
 * Ported from the ras_initiator reference (get_distance / median_inplace).
 */
#include "distance.h"

#include <math.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app_cs, CONFIG_LOG_DEFAULT_LEVEL);

#define DE_SLIDING_WINDOW_SIZE 9

struct de_buffer {
	cs_de_dist_estimates_t estimates[DE_SLIDING_WINDOW_SIZE];
	uint8_t num_valid;
	uint8_t index;
};

static struct de_buffer buffers[DE_MAX_AP];
static K_MUTEX_DEFINE(buffers_mutex);

void distance_reset(void)
{
	k_mutex_lock(&buffers_mutex, K_FOREVER);
	memset(buffers, 0, sizeof(buffers));
	k_mutex_unlock(&buffers_mutex);
}

void distance_store(uint8_t ap, const cs_de_dist_estimates_t *estimates)
{
	if (ap >= DE_MAX_AP) {
		return;
	}

	k_mutex_lock(&buffers_mutex, K_FOREVER);

	struct de_buffer *b = &buffers[ap];

	memcpy(&b->estimates[b->index], estimates, sizeof(*estimates));
	b->index = (b->index + 1) % DE_SLIDING_WINDOW_SIZE;
	if (b->num_valid < DE_SLIDING_WINDOW_SIZE) {
		b->num_valid++;
	}

	k_mutex_unlock(&buffers_mutex);
}

uint8_t distance_count(uint8_t ap)
{
	if (ap >= DE_MAX_AP) {
		return 0;
	}
	return buffers[ap].num_valid;
}

static int float_cmp(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;

	return (fa > fb) - (fa < fb);
}

static float median_inplace(int count, float *values)
{
	if (count == 0) {
		return NAN;
	}

	qsort(values, count, sizeof(float), float_cmp);

	if (count % 2 == 0) {
		return (values[count / 2] + values[count / 2 - 1]) / 2;
	}
	return values[count / 2];
}

cs_de_dist_estimates_t distance_get(uint8_t ap)
{
	cs_de_dist_estimates_t out = {0};
	float tmp_ifft[DE_SLIDING_WINDOW_SIZE];
	float tmp_phase[DE_SLIDING_WINDOW_SIZE];
	float tmp_rtt[DE_SLIDING_WINDOW_SIZE];
	uint8_t n_ifft = 0, n_phase = 0, n_rtt = 0;

	if (ap >= DE_MAX_AP) {
		out.ifft = out.phase_slope = out.rtt = out.best = NAN;
		return out;
	}

	k_mutex_lock(&buffers_mutex, K_FOREVER);

	struct de_buffer *b = &buffers[ap];

	for (uint8_t i = 0; i < b->num_valid; i++) {
		if (isfinite(b->estimates[i].ifft)) {
			tmp_ifft[n_ifft++] = b->estimates[i].ifft;
		}
		if (isfinite(b->estimates[i].phase_slope)) {
			tmp_phase[n_phase++] = b->estimates[i].phase_slope;
		}
		if (isfinite(b->estimates[i].rtt)) {
			tmp_rtt[n_rtt++] = b->estimates[i].rtt;
		}
	}

	k_mutex_unlock(&buffers_mutex);

	out.ifft = median_inplace(n_ifft, tmp_ifft);
	out.phase_slope = median_inplace(n_phase, tmp_phase);
	out.rtt = median_inplace(n_rtt, tmp_rtt);
	out.best = NAN;

	return out;
}
