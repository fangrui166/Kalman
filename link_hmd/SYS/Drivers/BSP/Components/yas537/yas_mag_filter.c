/*
 * CONFIDENTIAL
 * Copyright (c) 2013-2015 Yamaha Corporation
 */

#include "yas.h"
#if YAS_MAG_FILTER_ENABLE

struct yas_adap_filter {
	uint8_t num;
	uint8_t index;
	uint8_t filter_len;
	int32_t filter_noise;
	int16_t sequence[YAS_MAG_DEFAULT_FILTER_LEN];
};
struct yas_thresh_filter {
	uint16_t threshold;
	int32_t *p, last;
};

struct yas_filter_module {
	struct yas_adap_filter adap_filter[3];
	struct yas_thresh_filter thresh_filter[3];
	struct yas_mag_filter_config config;
	int initialized;
};

static struct yas_filter_module module;

#define square(a)		((a) * (a))

static int32_t yas_adap_filter_update(struct yas_adap_filter *adap_filter,
		int32_t in)
{
	int32_t avg, sum;
	int i;
	if (adap_filter->filter_len <= 1)
		return in;
	if (adap_filter->num < adap_filter->filter_len) {
		adap_filter->sequence[adap_filter->index++]
			= (int16_t)(in / 100);
		adap_filter->num++;
		return in;
	}
	if (adap_filter->filter_len <= adap_filter->index)
		adap_filter->index = 0;
	adap_filter->sequence[adap_filter->index++] = (int16_t)(in / 100);
	avg = 0;
	for (i = 0; i < adap_filter->filter_len; i++)
		avg += adap_filter->sequence[i];
	avg /= adap_filter->filter_len;
	sum = 0;
	for (i = 0; i < adap_filter->filter_len; i++)
		sum += square(avg - adap_filter->sequence[i]);
	sum /= adap_filter->filter_len;
	if (sum <= adap_filter->filter_noise)
		return avg * 100;
	return ((in/100 - avg) * (sum - adap_filter->filter_noise) / sum + avg)
		* 100;
}

static void yas_adap_filter_init(struct yas_adap_filter *adap_filter,
		uint8_t len, uint16_t noise)
{
	int i;
	adap_filter->num = 0;
	adap_filter->index = 0;
	adap_filter->filter_noise = (noise / 100) * (noise / 100);
	adap_filter->filter_len = len;
	for (i = 0; i < adap_filter->filter_len; ++i)
		adap_filter->sequence[i] = 0;
}

static int32_t yas_thresh_filter_update(struct yas_thresh_filter
		*thresh_filter, int32_t in)
{
	if (thresh_filter->p == NULL) {
		thresh_filter->p = &thresh_filter->last;
		thresh_filter->last = in;
	}
	if (thresh_filter->threshold < ABS(*thresh_filter->p - in)) {
		*thresh_filter->p = in;
		return in;
	} else
		return *thresh_filter->p;
}

static void yas_thresh_filter_init(struct yas_thresh_filter *thresh_filter,
		uint16_t threshold)
{
	thresh_filter->threshold = threshold;
	thresh_filter->p = NULL;
	thresh_filter->last = 0;
}


static int yas_update(struct yas_vector *input, struct yas_vector *output)
{
	int i;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (input == NULL || output == NULL)
		return YAS_ERROR_ARG;
	for (i = 0; i < 3; i++) {
		output->v[i] = yas_adap_filter_update(&module.adap_filter[i],
				input->v[i]);
		output->v[i] = yas_thresh_filter_update(
				&module.thresh_filter[i], output->v[i]);
	}
	return YAS_NO_ERROR;
}

static int yas_get_config(struct yas_mag_filter_config *c)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (c == NULL)
		return YAS_ERROR_ARG;
	*c = module.config;
	return YAS_NO_ERROR;
}

static int yas_set_config(struct yas_mag_filter_config *c)
{
	int i;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (c == NULL)
		return YAS_ERROR_ARG;
	if (YAS_MAG_DEFAULT_FILTER_LEN < c->len)
		return YAS_ERROR_ARG;
	for (i = 0; i < 3; i++) {
		yas_adap_filter_init(&module.adap_filter[i],
				c->len, c->noise);
		yas_thresh_filter_init(&module.thresh_filter[i],
				c->threshold);
	}
	module.config = *c;
	return YAS_NO_ERROR;
}

static int yas_init(void)
{
	uint16_t noise;
	int i;
	if (module.initialized)
		return YAS_ERROR_INITIALIZE;
	noise = YAS_MAG_DEFAULT_FILTER_NOISE;
	module.config.len = YAS_MAG_DEFAULT_FILTER_LEN;
	module.config.threshold = YAS_MAG_DEFAULT_FILTER_THRESH;
	for (i = 0; i < 3; i++) {
		module.config.noise = noise;
		yas_adap_filter_init(&module.adap_filter[i],
				YAS_MAG_DEFAULT_FILTER_LEN, noise);
		yas_thresh_filter_init(&module.thresh_filter[i],
				YAS_MAG_DEFAULT_FILTER_THRESH);
	}
	module.initialized = 1;
	return YAS_NO_ERROR;
}

static int yas_term(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	module.initialized = 0;
	return YAS_NO_ERROR;
}

static int yas_reset(void)
{
	struct yas_mag_filter_config config;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	config = module.config;
	yas_term();
	yas_init();
	yas_set_config(&config);
	return YAS_NO_ERROR;
}

int yas_mag_filter_init(struct yas_mag_filter *f)
{
	if (f == NULL)
		return YAS_ERROR_ARG;
	f->init = yas_init;
	f->term = yas_term;
	f->reset = yas_reset;
	f->get_config = yas_get_config;
	f->set_config = yas_set_config;
	f->update = yas_update;
	module.initialized = 0;
	return YAS_NO_ERROR;
}
#endif
