/*
 * Copyright (c) 2013-2014 Yamaha Corporation
 * CONFIDENTIAL
 */
#include "yas_algo.h"

#define YAS_MAX_MAG_OFFSET_SHIFT (26754)
#define YAS_MAX_SQ_MAG_OFFSET_SHIFT (715776516)

#if 0
static const struct yas_matrix unit_matrix
= { {YAS_MATRIX_NORM, 0, 0, 0, YAS_MATRIX_NORM, 0, 0, 0, YAS_MATRIX_NORM} };
#endif
static const struct yas_vector zero_vector = { {0, 0, 0} };

struct sensor_algo {
#if YAS_MAG_CALIB_ENABLE
	struct yas_mag_calib mcalib;
#endif
#if YAS_MAG_FILTER_ENABLE
	struct yas_mag_filter mfilter;
#endif
#if YAS_GYRO_CALIB_ENABLE
	struct yas_gyro_calib gcalib;
#endif
#if YAS_FUSION_ENABLE
	struct yas_fusion fusion;
	uint8_t fusion_enable;
	int32_t sq_mag_offset_change;
#endif
#if YAS_SOFTWARE_GYROSCOPE_ENABLE
	struct yas_swgyro swgyro;
	uint8_t swgyro_enable;
#endif
#if YAS_STEPCOUNTER_ENABLE
	struct yas_stepcounter stepcounter;
	uint8_t stepcounter_enable;
#endif
#if YAS_SIGNIFICANT_MOTION_ENABLE
	struct yas_sfm sfm;
	uint8_t sfm_enable;
#endif
#if YAS_LOG_ENABLE
	struct yas_log log;
	uint8_t log_enable;
	uint8_t log_initialized;
#endif
#if YAS_MAG_CALIB_ENABLE
	struct yas_vector uncal_mag;
#endif
#if YAS_GYRO_CALIB_ENABLE
	struct yas_vector uncal_gyro;
#endif
	struct yas_algo_state state;
	uint8_t filter_enable[yas_num_sensors];
	uint8_t calib_enable[yas_num_sensors];
	uint8_t initialized;
};

static struct sensor_algo module;

#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
static void apply_matrix(struct yas_vector *v, struct yas_matrix *m)
{
	int32_t tmp[3];
	int i;
	if (m == NULL)
		return;
	for (i = 0; i < 3; i++)
		tmp[i] = (int32_t)(((int64_t) m->m[i*3+0] * v->v[0]
					+ (int64_t) m->m[i*3+1] * v->v[1]
					+ (int64_t) m->m[i*3+2] * v->v[2])
				/ 10000);
	for (i = 0; i < 3; i++)
		v->v[i] = tmp[i];
}
#endif

#if YAS_FUSION_ENABLE
/* Returs value will be clipped to YAS_MAX_SQ_MAG_OFFSET_SHIFT. */
static int32_t yas_algo_calc_sq_offset_change(const struct yas_vector *prev,
		const struct yas_vector *current)
{
	int_fast8_t i;
	int32_t temp_i[3];
	int32_t sq_mag_offset_shift;

	sq_mag_offset_shift = 0;
	for (i = 2; i >= 0; --i)
		temp_i[i] = prev->v[i] - current->v[i];

	for (i = 2; i >= 0; --i) {
		if (temp_i[i] < -YAS_MAX_MAG_OFFSET_SHIFT ||
				temp_i[i] > YAS_MAX_MAG_OFFSET_SHIFT) {
			sq_mag_offset_shift = YAS_MAX_SQ_MAG_OFFSET_SHIFT;
			break;
		}
		sq_mag_offset_shift += temp_i[i] * temp_i[i];
		if (sq_mag_offset_shift > YAS_MAX_SQ_MAG_OFFSET_SHIFT) {
			sq_mag_offset_shift = YAS_MAX_SQ_MAG_OFFSET_SHIFT;
			break;
		}
	}
	return sq_mag_offset_shift;
}
#endif

int yas_algo_get_state(struct yas_algo_state *state)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (state == NULL)
		return YAS_ERROR_ARG;
	*state = module.state;
	return YAS_NO_ERROR;
}

int yas_algo_set_state(struct yas_algo_state *state)
{
	int rt;
#if YAS_FUSION_ENABLE
	int32_t sq_change;
#endif

	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (state == NULL)
		return YAS_ERROR_ARG;
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
	if (3 < state->accuracy[yas_accelerometer])
		return YAS_ERROR_ARG;
#endif

#if YAS_FUSION_ENABLE
	sq_change = yas_algo_calc_sq_offset_change(&module.state.offset[
			yas_magnetic], &state->offset[yas_magnetic]);
#endif

#if YAS_MAG_CALIB_ENABLE
	rt = module.mcalib.set_offset(YAS_TYPE_MAG,
			&state->offset[yas_magnetic],
			state->accuracy[yas_magnetic]);
	if (rt < 0)
		return rt;
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
	rt = module.mcalib.set_dynamic_matrix(&state->mag_dynamic_matrix);
	if (rt < 0)
		return rt;
#endif
#endif
#if YAS_GYRO_CALIB_ENABLE
	rt = module.gcalib.set_offset(YAS_TYPE_MAG,
			&state->offset[yas_magnetic],
			state->accuracy[yas_magnetic]);
	if (rt < 0)
		return rt;
	rt = module.gcalib.set_offset(YAS_TYPE_GYRO,
			&state->offset[yas_gyroscope],
			state->accuracy[yas_gyroscope]);
	if (rt < 0)
		return rt;
#endif
#if YAS_FUSION_ENABLE
	(void) module.fusion.notify_offset_change(sq_change);
#endif
	module.state = *state;
	return YAS_NO_ERROR;
}

int yas_algo_get_calib_enable(int32_t type)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_GYRO_CALIB_ENABLE
	if (type & YAS_TYPE_GYRO)
		return module.calib_enable[yas_gyroscope];
#endif
#if YAS_MAG_CALIB_ENABLE
	if (type & YAS_TYPE_MAG)
		return module.calib_enable[yas_magnetic];
#endif
	return YAS_ERROR_ARG;
}

int yas_algo_set_calib_enable(int32_t type, int enable)
{
	int result = YAS_NO_ERROR;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (type & YAS_TYPE_ACC)
		result = YAS_ERROR_ARG;
#if YAS_GYRO_CALIB_ENABLE
	if (type & YAS_TYPE_GYRO)
		module.calib_enable[yas_gyroscope] = !!enable;
#endif
#if YAS_MAG_CALIB_ENABLE
	if (type & YAS_TYPE_MAG)
		module.calib_enable[yas_magnetic] = !!enable;
#endif
	return result;
}

int yas_algo_get_calib_config(int32_t type, void *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_GYRO_CALIB_ENABLE
	if (type & YAS_TYPE_GYRO)
		return module.gcalib.get_config(
				(struct yas_gyro_calib_config *) config);
#endif
#if YAS_MAG_CALIB_ENABLE
	if (type & YAS_TYPE_MAG)
		return module.mcalib.get_config(
				(struct yas_mag_calib_config *) config);
#endif
	return YAS_ERROR_ARG;
}

int yas_algo_set_calib_config(int32_t type, void *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_GYRO_CALIB_ENABLE
	if (type & YAS_TYPE_GYRO)
		return module.gcalib.set_config(
				(struct yas_gyro_calib_config *) config);
#endif
#if YAS_MAG_CALIB_ENABLE
	if (type & YAS_TYPE_MAG) {
		int rt;
		struct yas_mag_calib_config *p;
		p = (struct yas_mag_calib_config *)config;
		rt = module.mcalib.set_config(p);
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
		if (p != NULL && p->mode != YAS_MAG_CALIB_MODE_ELLIPSOID
			&& p->mode != YAS_MAG_CALIB_MODE_ELLIPSOID_WITH_GYRO)
			module.state.mag_dynamic_matrix = unit_matrix;
#endif
		return rt;
	}
#endif
	return YAS_ERROR_ARG;
}

int yas_algo_get_calib_result(int32_t type, void *result)
{
	int rt;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_GYRO_CALIB_ENABLE
	if (type & YAS_TYPE_GYRO) {
		struct yas_gyro_calib_result *gr
			= (struct yas_gyro_calib_result *) result;
		rt =  module.gcalib.get_result(gr);
		if (rt < 0)
			return rt;
		return YAS_NO_ERROR;
	}
#endif
#if YAS_MAG_CALIB_ENABLE
	if (type & YAS_TYPE_MAG) {
		struct yas_mag_calib_result *gm
			= (struct yas_mag_calib_result *) result;
		rt = module.mcalib.get_result(gm);
		if (rt < 0)
			return rt;
		return YAS_NO_ERROR;
	}
#endif
	return YAS_ERROR_ARG;
}

int yas_algo_reset_calib(int32_t type)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
	if (type & YAS_TYPE_ACC) {
		module.state.offset[yas_accelerometer] = zero_vector;
		module.state.accuracy[yas_accelerometer] = 0;
	}
#endif
	if (type & YAS_TYPE_MAG) {
		module.state.offset[yas_magnetic] = zero_vector;
		module.state.accuracy[yas_magnetic] = 0;
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
		module.state.mag_dynamic_matrix = unit_matrix;
#endif
#if YAS_MAG_CALIB_ENABLE
		module.mcalib.reset();
#endif
	}
#if YAS_GYRO_DRIVER != YAS_GYRO_DRIVER_NONE
	if (type & YAS_TYPE_GYRO) {
		module.state.offset[yas_gyroscope] = zero_vector;
		module.state.accuracy[yas_gyroscope] = 0;
#if YAS_GYRO_CALIB_ENABLE
		module.gcalib.reset();
#endif
	}
#endif
	return YAS_NO_ERROR;
}

int yas_algo_get_filter_enable(int32_t type)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_MAG_FILTER_ENABLE
	if (type & YAS_TYPE_MAG)
		return module.filter_enable[yas_magnetic];
#endif
	return YAS_ERROR_ARG;
}

int yas_algo_set_filter_enable(int32_t type, int enable)
{
	int result = YAS_NO_ERROR;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (type & YAS_TYPE_GYRO)
		result = YAS_ERROR_ARG;
	if (type & YAS_TYPE_ACC)
		result = YAS_ERROR_ARG;
#if YAS_MAG_FILTER_ENABLE
	if (type & YAS_TYPE_MAG) {
		if (!module.filter_enable[yas_magnetic] && enable)
			module.mfilter.reset();
		module.filter_enable[yas_magnetic] = !!enable;
	}
#endif
	return result;
}

int yas_algo_get_filter_config(int32_t type, void *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_MAG_FILTER_ENABLE
	if (type & YAS_TYPE_MAG)
		return module.mfilter.get_config(
				(struct yas_mag_filter_config *) config);
#endif
	return YAS_ERROR_ARG;
}

int yas_algo_set_filter_config(int32_t type, void *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_MAG_FILTER_ENABLE
	if (type & YAS_TYPE_MAG)
		return module.mfilter.set_config(
				(struct yas_mag_filter_config *) config);
#endif
	return YAS_ERROR_ARG;
}

int yas_algo_reset_filter(int32_t type)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_MAG_FILTER_ENABLE
	if (type & YAS_TYPE_MAG)
		return module.mfilter.reset();
#endif
	return YAS_ERROR_ARG;
}

#if YAS_FUSION_ENABLE
int yas_algo_get_fusion_enable(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.fusion_enable;
}

int yas_algo_set_fusion_enable(int enable)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (!module.fusion_enable && enable)
		module.fusion.reset();
	module.fusion_enable = !!enable;
	return YAS_NO_ERROR;
}

int yas_algo_get_fusion_config(struct yas_fusion_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.fusion.get_config(config);
}

int yas_algo_set_fusion_config(struct yas_fusion_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.fusion.set_config(config);
}

int yas_algo_get_fusion_result(struct yas_fusion_result *result)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.fusion.get_result(result);
}

int yas_algo_reset_fusion(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	module.sq_mag_offset_change = YAS_MAX_SQ_MAG_OFFSET_SHIFT;
	return module.fusion.reset();
}
#endif

#if YAS_SOFTWARE_GYROSCOPE_ENABLE
int yas_algo_get_swgyro_enable(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.swgyro_enable;
}

int yas_algo_set_swgyro_enable(int enable)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (!module.swgyro_enable && enable)
		module.swgyro.reset();
	module.swgyro_enable = !!enable;
	return YAS_NO_ERROR;
}

int yas_algo_get_swgyro_config(struct yas_swgyro_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.swgyro.get_config(config);
}

int yas_algo_set_swgyro_config(struct yas_swgyro_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.swgyro.set_config(config);
}

int yas_algo_get_swgyro_result(struct yas_swgyro_result *result)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.swgyro.get_result(result);
}

int yas_algo_reset_swgyro(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.swgyro.reset();
}
#endif

#if YAS_STEPCOUNTER_ENABLE
int yas_algo_get_stepcounter_enable(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.stepcounter_enable;
}

int yas_algo_set_stepcounter_enable(int enable)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if 0
	if (!module.stepcounter_enable && enable)
		module.stepcounter.reset();
#endif
	module.stepcounter_enable = !!enable;
	return YAS_NO_ERROR;
}

int yas_algo_get_stepcounter_config(struct yas_stepcounter_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.stepcounter.get_config(config);
}

int yas_algo_set_stepcounter_config(struct yas_stepcounter_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.stepcounter.set_config(config);
}

int yas_algo_get_stepcounter_result(struct yas_stepcounter_result *result)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.stepcounter.get_result(result);
}

int yas_algo_reset_stepcounter(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.stepcounter.reset();
}
#endif

#if YAS_SIGNIFICANT_MOTION_ENABLE
int yas_algo_get_sfm_enable(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.sfm_enable;
}

int yas_algo_set_sfm_enable(int enable)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if 0
	if (!module.sfm_enable && enable)
		module.sfm.reset();
#endif
	module.sfm_enable = !!enable;
	return YAS_NO_ERROR;
}

int yas_algo_get_sfm_config(struct yas_sfm_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.sfm.get_config(config);
}

int yas_algo_set_sfm_config(struct yas_sfm_config *config)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.sfm.set_config(config);
}

int yas_algo_get_sfm_result(struct yas_sfm_result *result)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.sfm.get_result(result);
}

int yas_algo_reset_sfm(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.sfm.reset();
}
#endif

int yas_algo_update(struct yas_data *raw, struct yas_data *calibrated,
		int num)
{
	struct yas_algo_state *s = &module.state;
	int_fast8_t i, j;
	int rt, result = 0;
#if YAS_FUSION_ENABLE
	struct yas_vector prev_mag_offset;
#endif

	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (raw == NULL || calibrated == NULL || num < 0)
		return YAS_ERROR_ARG;
	if (num == 0)
		return YAS_NO_ERROR;

#if YAS_FUSION_ENABLE
	prev_mag_offset = s->offset[yas_magnetic];
#endif

#if YAS_GYRO_CALIB_ENABLE
	if (module.calib_enable[yas_gyroscope]) {
		rt = module.gcalib.update(raw, num);
		if (0 < rt) {
			module.gcalib.get_offset(YAS_TYPE_GYRO,
					&s->offset[yas_gyroscope],
					&s->accuracy[yas_gyroscope]);
			result |= YAS_GYRO_CALIB_UPDATE;
		}
	}
#endif

#if YAS_MAG_CALIB_ENABLE
	if (module.calib_enable[yas_magnetic]) {
		rt = module.mcalib.update(raw, num);
		if (0 < rt) {
			module.mcalib.get_offset(YAS_TYPE_MAG,
					&s->offset[yas_magnetic],
					&s->accuracy[yas_magnetic]);
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
			module.mcalib.get_dynamic_matrix(
					&s->mag_dynamic_matrix);
#endif
			result |= YAS_MAG_CALIB_UPDATE;
		}
	}
#endif

	for (i = 0; i < num; ++i) {
		calibrated[i].type = raw[i].type;
		calibrated[i].timestamp = raw[i].timestamp;
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
		if (raw[i].type == YAS_TYPE_ACC) {
			for (j = 0; j < 3; j++) {
				calibrated[i].xyz.v[j] = raw[i].xyz.v[j]
					- s->offset[yas_accelerometer].v[j];
			}
			calibrated[i].accuracy
				= s->accuracy[yas_accelerometer];
		}
#endif

		if (raw[i].type == YAS_TYPE_MAG) {
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
			struct yas_mag_calib_config cfg;
			module.mcalib.get_config(&cfg);
#endif
			for (j = 0; j < 3; j++){
				calibrated[i].caliboffset.v[j] = s->offset[yas_magnetic].v[j];
				calibrated[i].xyz.v[j] = raw[i].xyz.v[j]
					- s->offset[yas_magnetic].v[j];
			}
			calibrated[i].accuracy = s->accuracy[yas_magnetic];
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
			if (cfg.mode == YAS_MAG_CALIB_MODE_ELLIPSOID
				|| cfg.mode ==
				YAS_MAG_CALIB_MODE_ELLIPSOID_WITH_GYRO) {
				apply_matrix(&calibrated[i].xyz,
						&s->mag_dynamic_matrix);
			}
#endif
		}

#if YAS_GYRO_DRIVER != YAS_GYRO_DRIVER_NONE
		if (raw[i].type == YAS_TYPE_GYRO) {
			for (j = 0; j < 3; j++) {
				calibrated[i].xyz.v[j] = raw[i].xyz.v[j]
					- s->offset[yas_gyroscope].v[j];
			}
			calibrated[i].accuracy = s->accuracy[yas_gyroscope];
		}
#endif
	}

#if YAS_SOFTWARE_GYROSCOPE_ENABLE
	if (module.swgyro_enable)
		module.swgyro.update(calibrated, num);
#endif

#if YAS_FUSION_ENABLE
	if (module.fusion_enable) {
		int32_t sq_mag_offset_shift;
		sq_mag_offset_shift =
			yas_algo_calc_sq_offset_change(&prev_mag_offset,
					&s->offset[yas_magnetic]);
		if (sq_mag_offset_shift > 0)
			module.fusion.notify_offset_change(sq_mag_offset_shift);
		module.fusion.update(calibrated, num);
	}
#endif

#if YAS_STEPCOUNTER_ENABLE
	if (module.stepcounter_enable) {
		rt = module.stepcounter.update(calibrated, num);
		rt &= ~0x10; /* Do not notify NOTIFY_EVT_NONE */
		if (rt)
			result |= YAS_STEPCOUNTER_UPDATE;
	}
#endif
#if YAS_SIGNIFICANT_MOTION_ENABLE
	if (module.sfm_enable) {
		rt = module.sfm.update(calibrated, num);
		if (rt)
			result |= YAS_SIGNIFICANT_MOTION_UPDATE;
	}
#endif
#if YAS_MAG_FILTER_ENABLE
	for (i = 0; i < num; i++) {
		if (calibrated[i].type == YAS_TYPE_MAG
				&& module.filter_enable[yas_magnetic])
			module.mfilter.update(&calibrated[i].xyz,
					&calibrated[i].xyz);
	}
#endif

#if YAS_LOG_ENABLE
	if (module.log_enable) {
		char buf[sizeof("-2147483648")*37];
		uint32_t tm = 0;
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
		struct yas_data *acc = NULL, *acc2 = NULL;
#endif
		struct yas_data *mag = NULL, *mag2 = NULL;
#if YAS_GYRO_DRIVER != YAS_GYRO_DRIVER_NONE
		struct yas_data *gyro = NULL, *gyro2 = NULL;
#endif
		struct yas_mag_calib_result r;
		module.mcalib.get_result(&r);
		for (i = 0; i < num; i++) {
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
			if (raw[i].type == YAS_TYPE_ACC) {
				acc = &raw[i];
				acc2 = &calibrated[i];
				tm = raw[i].timestamp;
			}
#endif
			if (raw[i].type == YAS_TYPE_MAG) {
				mag = &raw[i];
				mag2 = &calibrated[i];
				tm = raw[i].timestamp;
			}
#if YAS_GYRO_DRIVER != YAS_GYRO_DRIVER_NONE
			if (raw[i].type == YAS_TYPE_GYRO) {
				gyro = &raw[i];
				gyro2 = &calibrated[i];
				tm = raw[i].timestamp;
			}
#endif
		}
		sprintf(buf, "%u,%d,"
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
				"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
#endif
				"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
#if YAS_GYRO_DRIVER != YAS_GYRO_DRIVER_NONE
				"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
#endif
				"%d,%d,%d,%d,%d,\n",
				tm, result,
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
				acc == NULL ? 0 : acc->xyz.v[0],
				acc == NULL ? 0 : acc->xyz.v[1],
				acc == NULL ? 0 : acc->xyz.v[2],
				s->offset[yas_accelerometer].v[0],
				s->offset[yas_accelerometer].v[1],
				s->offset[yas_accelerometer].v[2],
				s->accuracy[yas_accelerometer],
				acc2 == NULL ? 0 : acc2->xyz.v[0],
				acc2 == NULL ? 0 : acc2->xyz.v[1],
				acc2 == NULL ? 0 : acc2->xyz.v[2],
#endif
				mag == NULL ? 0 : mag->xyz.v[0],
				mag == NULL ? 0 : mag->xyz.v[1],
				mag == NULL ? 0 : mag->xyz.v[2],
				s->offset[yas_magnetic].v[0],
				s->offset[yas_magnetic].v[1],
				s->offset[yas_magnetic].v[2],
				s->accuracy[yas_magnetic],
				mag2 == NULL ? 0 : mag2->xyz.v[0],
				mag2 == NULL ? 0 : mag2->xyz.v[1],
				mag2 == NULL ? 0 : mag2->xyz.v[2],
#if YAS_GYRO_DRIVER != YAS_GYRO_DRIVER_NONE
				gyro == NULL ? 0 : gyro->xyz.v[0],
				gyro == NULL ? 0 : gyro->xyz.v[1],
				gyro == NULL ? 0 : gyro->xyz.v[2],
				s->offset[yas_gyroscope].v[0],
				s->offset[yas_gyroscope].v[1],
				s->offset[yas_gyroscope].v[2],
				s->accuracy[yas_gyroscope],
				gyro2 == NULL ? 0 : gyro2->xyz.v[0],
				gyro2 == NULL ? 0 : gyro2->xyz.v[1],
				gyro2 == NULL ? 0 : gyro2->xyz.v[2],
#endif
				r.spread, r.variation, r.radius, r.axis,
				r.level);
		module.log.log_write(buf, (int)strlen(buf));
	}
#endif
	return result;
}

#if YAS_LOG_ENABLE
int yas_algo_log_init(struct yas_log *log)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (log == NULL || log->log_open == NULL || log->log_close == NULL
			|| log->log_write == NULL)
		return YAS_ERROR_ARG;
	module.log = *log;
	module.log_initialized = 1;
	return YAS_NO_ERROR;
}

int yas_algo_get_log_enable(void)
{
	if (!module.initialized || !module.log_initialized)
		return YAS_ERROR_INITIALIZE;
	return module.log_enable;
}

int yas_algo_set_log_enable(int enable)
{
	const char *header
		= "time,update,"
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
		"axn,ayn,azn,aox,aoy,aoz,acc,arx,ary,arz,"
#endif
		"mxn,myn,mzn,mox,moy,moz,mcc,mrx,mry,mrz,"
#if YAS_GYRO_DRIVER != YAS_GYRO_DRIVER_NONE
		"gxn,gyn,gzn,gox,goy,goz,gcc,grx,gry,grz,"
#endif
		"spd,var,rad,axs,lvl,\n";
	if (!module.initialized || !module.log_initialized)
		return YAS_ERROR_INITIALIZE;
	if (!module.log_enable && enable) {
		module.log.log_open();
		module.log.log_write(header, (int)strlen(header));
	}
	if (module.log_enable && !enable)
		module.log.log_close();
	module.log_enable = !!enable;
	return YAS_NO_ERROR;
}
#endif

int yas_algo_init(void)
{
	struct yas_algo_state *s = &module.state;
	int i;
	if (module.initialized)
		return YAS_ERROR_INITIALIZE;

#if YAS_MAG_CALIB_ENABLE
	yas_mag_calib_init(&module.mcalib);
	module.mcalib.init();
#endif
#if YAS_MAG_FILTER_ENABLE
	yas_mag_filter_init(&module.mfilter);
	module.mfilter.init();
#endif
#if YAS_GYRO_CALIB_ENABLE
	yas_gyro_calib_init(&module.gcalib);
	module.gcalib.init();
#endif
#if YAS_FUSION_ENABLE
	yas_fusion_init(&module.fusion);
	module.fusion.init();
	module.fusion_enable = 0;
	module.sq_mag_offset_change = YAS_MAX_SQ_MAG_OFFSET_SHIFT;
#endif
#if YAS_SOFTWARE_GYROSCOPE_ENABLE
	yas_swgyro_init(&module.swgyro);
	module.swgyro.init();
	module.swgyro_enable = 0;
#endif
#if YAS_STEPCOUNTER_ENABLE
	yas_stepcounter_init(&module.stepcounter);
	module.stepcounter.init();
	module.stepcounter_enable = 0;
#endif
#if YAS_SIGNIFICANT_MOTION_ENABLE
	yas_sfm_init(&module.sfm);
	module.sfm.init();
	module.sfm_enable = 0;
#endif
	for (i = 0; i < yas_num_sensors; i++) {
		module.filter_enable[i] = 0;
		module.calib_enable[i] = 0;
	}
#if YAS_ACC_DRIVER != YAS_ACC_DRIVER_NONE
	s->offset[yas_accelerometer] = zero_vector;
	s->accuracy[yas_accelerometer] = 0;
#endif
#if YAS_MAG_CALIB_ENABLE
	module.mcalib.get_offset(YAS_TYPE_MAG, &s->offset[yas_magnetic],
			&s->accuracy[yas_magnetic]);
#if YAS_MAG_CALIB_ELLIPSOID_ENABLE
	module.mcalib.get_dynamic_matrix(
			&s->mag_dynamic_matrix);
#endif
#endif
#if YAS_GYRO_CALIB_ENABLE
	module.gcalib.get_offset(YAS_TYPE_GYRO, &s->offset[yas_gyroscope],
			&s->accuracy[yas_gyroscope]);
#endif
#if YAS_MAG_CALIB_ENABLE
	module.uncal_mag = zero_vector;
#endif
#if YAS_GYRO_CALIB_ENABLE
	module.uncal_gyro = zero_vector;
#endif
	module.initialized = 1;
	return YAS_NO_ERROR;
}

int yas_algo_term(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_MAG_CALIB_ENABLE
	module.mcalib.term();
#endif
#if YAS_MAG_FILTER_ENABLE
	module.mfilter.term();
#endif
#if YAS_GYRO_CALIB_ENABLE
	module.gcalib.term();
#endif
#if YAS_FUSION_ENABLE
	module.fusion.term();
#endif
#if YAS_SOFTWARE_GYROSCOPE_ENABLE
	module.swgyro.term();
#endif
#if YAS_STEPCOUNTER_ENABLE
	module.stepcounter.term();
#endif
#if YAS_SIGNIFICANT_MOTION_ENABLE
	module.sfm.term();
#endif
#if YAS_LOG_ENABLE
	module.log_enable = 0;
	module.log_initialized = 0;
#endif
	module.initialized = 0;
	return YAS_NO_ERROR;
}
