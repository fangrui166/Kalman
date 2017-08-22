/*
 * Copyright (c) 2013 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#include "yas_drv.h"

struct sensor_driver {
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	struct yas_gyro_driver gyro;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	struct yas_acc_driver acc;
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	struct yas_mag_driver mag;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	struct yas_acc_mag_driver am;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	struct yas_acc_gyro_driver ag;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	struct yas_acc_mag_gyro_driver amg;
#endif
#if YAS_LOG_ENABLE
	int log_enable;
	struct yas_log log;
	uint8_t log_initialized;
#endif
	int initialized;
};

static struct sensor_driver driver;

int yas_driver_get_enable(int32_t type)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if (type & YAS_TYPE_GYRO)
		return driver.gyro.get_enable();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if (type & YAS_TYPE_ACC)
		return driver.acc.get_enable();
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if (type & YAS_TYPE_MAG)
		return driver.mag.get_enable();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_MAG))
		return driver.am.get_enable(type);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_GYRO))
		return driver.ag.get_enable(type);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	return driver.amg.get_enable(type);
#endif
	return YAS_ERROR_ARG;
}

int32_t yas_driver_set_enable(int32_t type, int enable)
{
	int rt;
	int32_t result = 0;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if (type & YAS_TYPE_GYRO) {
		rt = driver.gyro.set_enable(enable);
		if (rt == YAS_NO_ERROR)
			result |= YAS_TYPE_GYRO;
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if (type & YAS_TYPE_ACC) {
		rt = driver.acc.set_enable(enable);
		if (rt == YAS_NO_ERROR)
			result |= YAS_TYPE_ACC;
	}
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if (type & YAS_TYPE_MAG) {
		rt = driver.mag.set_enable(enable);
		if (rt == YAS_NO_ERROR)
			result |= YAS_TYPE_MAG;
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if (type & YAS_TYPE_ACC || type & YAS_TYPE_MAG) {
		rt = driver.am.set_enable(type, enable);
		if (rt == YAS_NO_ERROR)
			result |= (type & (YAS_TYPE_ACC|YAS_TYPE_MAG));
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if (type & YAS_TYPE_ACC || type & YAS_TYPE_GYRO) {
		rt = driver.ag.set_enable(type, enable);
		if (rt == YAS_NO_ERROR)
			result |= (type & (YAS_TYPE_ACC|YAS_TYPE_GYRO));
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	rt = driver.amg.set_enable(type, enable);
	if (rt == YAS_NO_ERROR)
		result |= (type & (YAS_TYPE_ACC|YAS_TYPE_MAG|YAS_TYPE_GYRO));
#endif
	return result;
}

int yas_driver_get_delay(int32_t type)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if (type & YAS_TYPE_GYRO)
		return driver.gyro.get_delay();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if (type & YAS_TYPE_ACC)
		return driver.acc.get_delay();
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if (type & YAS_TYPE_MAG)
		return driver.mag.get_delay();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_MAG))
		return driver.am.get_delay(type);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_GYRO))
		return driver.ag.get_delay(type);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	return driver.amg.get_delay(type);
#endif
	return YAS_ERROR_ARG;
}

int32_t yas_driver_set_delay(int32_t type, int delay)
{
	int rt;
	int32_t result = 0;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if (type & YAS_TYPE_GYRO) {
		rt = driver.gyro.set_delay(delay);
		if (rt == YAS_NO_ERROR)
			result |= YAS_TYPE_GYRO;
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if (type & YAS_TYPE_ACC) {
		rt = driver.acc.set_delay(delay);
		if (rt == YAS_NO_ERROR)
			result |= YAS_TYPE_ACC;
	}
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if (type & YAS_TYPE_MAG) {
		rt = driver.mag.set_delay(delay);
		if (rt == YAS_NO_ERROR)
			result |= YAS_TYPE_MAG;
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if (type & YAS_TYPE_ACC || type & YAS_TYPE_MAG) {
		rt = driver.am.set_delay(type, delay);
		if (rt == YAS_NO_ERROR)
			result |= (type & (YAS_TYPE_ACC|YAS_TYPE_MAG));
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if (type & YAS_TYPE_ACC || type & YAS_TYPE_GYRO) {
		rt = driver.ag.set_delay(type, delay);
		if (rt == YAS_NO_ERROR)
			result |= (type & (YAS_TYPE_ACC|YAS_TYPE_GYRO));
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	rt = driver.amg.set_delay(type, delay);
	if (rt == YAS_NO_ERROR)
		result |= (type & (YAS_TYPE_ACC|YAS_TYPE_MAG|YAS_TYPE_GYRO));
#endif
	return result;
}

int yas_driver_get_position(int32_t type)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if (type & YAS_TYPE_GYRO)
		return driver.gyro.get_position();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if (type & YAS_TYPE_ACC)
		return driver.acc.get_position();
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if (type & YAS_TYPE_MAG)
		return driver.mag.get_position();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_MAG))
		return driver.am.get_position();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_GYRO))
		return driver.ag.get_position();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	(void) type;
	return driver.amg.get_position();
#endif
	return YAS_ERROR_ARG;
}

int32_t yas_driver_set_position(int32_t type, int position)
{
	int rt;
	int32_t result = 0;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if (type & YAS_TYPE_GYRO) {
		rt = driver.gyro.set_position(position);
		if (rt == 0)
			result |= YAS_TYPE_GYRO;
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if (type & YAS_TYPE_ACC) {
		rt = driver.acc.set_position(position);
		if (rt == 0)
			result |= YAS_TYPE_ACC;
	}
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if (type & YAS_TYPE_MAG) {
		rt = driver.mag.set_position(position);
		if (rt == 0)
			result |= YAS_TYPE_MAG;
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_MAG)) {
		rt = driver.am.set_position(position);
		if (rt == 0)
			result |= (type & (YAS_TYPE_ACC|YAS_TYPE_MAG));
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_GYRO)) {
		rt = driver.ag.set_position(position);
		if (rt == 0)
			result |= (type & (YAS_TYPE_ACC|YAS_TYPE_GYRO));
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	(void) type;
	rt = driver.amg.set_position(position);
	if (rt == 0)
		result |= (type & (YAS_TYPE_ACC|YAS_TYPE_MAG|YAS_TYPE_GYRO));
#endif
	return result;
}

int yas_driver_measure(int32_t type, struct yas_data *data, int num)
{
	int rt, left = num;
	struct yas_data *p = data;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (data == NULL || num < 0)
		return YAS_ERROR_ARG;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if ((type & YAS_TYPE_GYRO) && 0 < left) {
		rt =  driver.gyro.measure(p, left);
		if (0 < rt) {
			p += rt;
			left -= rt;
		}
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if ((type & YAS_TYPE_ACC) && 0 < left) {
		rt =  driver.acc.measure(p, left);
		if (0 < rt) {
			p += rt;
			left -= rt;
		}
	}
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if ((type & YAS_TYPE_MAG) && 0 < left) {
		rt =  driver.mag.measure(p, left);
		if (0 < rt) {
			p += rt;
			left -= rt;
		}
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if ((type & YAS_TYPE_ACC || type & YAS_TYPE_MAG) && 0 < left) {
		rt =  driver.am.measure(type, p, left);
		if (0 < rt) {
			p += rt;
			left -= rt;
		}
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if ((type & YAS_TYPE_ACC || type & YAS_TYPE_GYRO) && 0 < left) {
		rt =  driver.ag.measure(type, p, left);
		if (0 < rt) {
			p += rt;
			left -= rt;
		}
	}
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	if ((type & YAS_TYPE_ACC || type & YAS_TYPE_MAG
				|| type & YAS_TYPE_GYRO) && 0 < left) {
		rt =  driver.amg.measure(type, p, left);
		if (0 < rt) {
			p += rt;
			left -= rt;
		}
	}
#endif
#if YAS_LOG_ENABLE
	if (driver.log_enable && 0 < num - left) {
		char buf[sizeof("-2147483648")*17];
		int8_t offset[3] = {0, 0, 0};
		uint16_t raw[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		struct yas_data *acc = NULL, *mag = NULL, *gyro = NULL;
		int i;
		uint32_t tm = 0;
		for (i = 0; i < num - left; i++) {
			if (data[i].type == YAS_TYPE_GYRO) {
				gyro = &data[i];
				tm = data[i].timestamp;
			}
			if (data[i].type == YAS_TYPE_ACC) {
				acc = &data[i];
				tm = data[i].timestamp;
			}
			if (data[i].type == YAS_TYPE_MAG) {
				mag = &data[i];
				tm = data[i].timestamp;
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
				driver.mag.ext(YAS532_GET_HW_OFFSET, offset);
				driver.mag.ext(YAS532_GET_LAST_RAWDATA, raw);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS535
				driver.mag.ext(YAS535_MAG_GET_HW_OFFSET,
						offset);
				driver.mag.ext(YAS535_GET_LAST_RAWDATA, raw);
#endif
			}
		}
		sprintf(buf,
			"%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
				tm,
				offset[0], offset[1], offset[2],
				raw[0], raw[1], raw[2], raw[3],
				acc == NULL ? 0 : acc->xyz.v[0],
				acc == NULL ? 0 : acc->xyz.v[1],
				acc == NULL ? 0 : acc->xyz.v[2],
				mag == NULL ? 0 : mag->xyz.v[0],
				mag == NULL ? 0 : mag->xyz.v[1],
				mag == NULL ? 0 : mag->xyz.v[2],
				gyro == NULL ? 0 : gyro->xyz.v[0],
				gyro == NULL ? 0 : gyro->xyz.v[1],
				gyro == NULL ? 0 : gyro->xyz.v[2]);
		driver.log.log_write(buf, (int)strlen(buf));
	}
#endif
	return num - left;
}

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS535
int yas_driver_get_state(struct yas_driver_state *state)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (state == NULL)
		return YAS_ERROR_ARG;
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	return driver.mag.ext(YAS532_GET_HW_OFFSET,
			state->mag_hard_offset);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS535
	return driver.am.ext(YAS535_MAG_GET_HW_OFFSET,
			state->mag_hard_offset);
#endif
}

int yas_driver_set_state(struct yas_driver_state *state)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (state == NULL)
		return YAS_ERROR_ARG;
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	return driver.mag.ext(YAS532_SET_HW_OFFSET,
			state->mag_hard_offset);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS535
	return driver.am.ext(YAS535_MAG_SET_HW_OFFSET,
			state->mag_hard_offset);
#endif
}
#endif

int yas_driver_ext(int32_t type, int32_t cmd, void *p)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	if (type & YAS_TYPE_GYRO)
		return driver.gyro.ext(cmd, p);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	if (type & YAS_TYPE_ACC)
		return driver.acc.ext(cmd, p);
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	if (type & YAS_TYPE_MAG)
		return driver.mag.ext(cmd, p);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_MAG))
		return driver.am.ext(cmd, p);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	if ((type & YAS_TYPE_ACC) || (type & YAS_TYPE_GYRO))
		return driver.ag.ext(cmd, p);
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	return driver.amg.ext(cmd, p);
#endif
	return YAS_ERROR_ARG;
}

#if YAS_LOG_ENABLE
int yas_driver_log_init(struct yas_log *log)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (log == NULL || log->log_open == NULL || log->log_close == NULL
			|| log->log_write == NULL)
		return YAS_ERROR_ARG;
	driver.log = *log;
	driver.log_initialized = 1;
	return YAS_NO_ERROR;
}

int yas_driver_get_log_enable(void)
{
	if (!driver.initialized || !driver.log_initialized)
		return YAS_ERROR_INITIALIZE;
	return driver.log_enable;
}

int yas_driver_set_log_enable(int enable)
{
	const char *header
		= "time,hox,hoy1,hoy2,x,y1,y2,t,axn,ayn,azn,mxn,myn,mzn,"
		"gxn,gyn,gzn\n";
	if (!driver.initialized || !driver.log_initialized)
		return YAS_ERROR_INITIALIZE;
	if (!driver.log_enable && enable) {
		driver.log.log_open();
		driver.log.log_write(header, (int)strlen(header));
	}
	if (driver.log_enable && !enable)
		driver.log.log_close();
	driver.log_enable = !!enable;
	return YAS_NO_ERROR;
}
#endif

int yas_driver_init(struct yas_driver_callback *f )
{
	int rt, result = YAS_NO_ERROR;
	//struct yas_driver_callback *f = NULL;
	//f=(struct yas_driver_callback *)handle->pExtVTable
	if (driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (f == NULL || f->device_open == NULL
			|| f->device_close == NULL
			|| f->device_read == NULL
			|| f->device_write == NULL
			|| f->usleep == NULL)
		return YAS_ERROR_ARG;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	driver.gyro.callback = *f;
	yas_gyro_driver_init(&driver.gyro);
	rt = driver.gyro.init();
	if (rt < 0)
		result = rt;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	driver.acc.callback = *f;
	yas_acc_driver_init(&driver.acc);
	rt = driver.acc.init();
	if (rt < 0)
		result = rt;
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	driver.mag.callback = *f;
	yas_mag_driver_init(&driver.mag);
	rt = driver.mag.init();
	if (rt < 0)
		result = rt;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	driver.am.callback = *f;
	yas_acc_mag_driver_init(&driver.am);
	rt = driver.am.init();
	if (rt < 0)
		result = rt;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	driver.ag.callback = *f;
	yas_acc_gyro_driver_init(&driver.ag);
	rt = driver.ag.init();
	if (rt < 0)
		result = rt;
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	driver.amg.callback = *f;
	yas_acc_mag_gyro_driver_init(&driver.amg);
	rt = driver.amg.init();
	if (rt < 0)
		result = rt;
#endif
	driver.initialized = 1;
	return result;
}

int yas_driver_term(void)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
#if YAS_TYPE_GYRO == YAS_TYPE_G_GYRO
	driver.gyro.term();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_A_ACC
	driver.acc.term();
#endif
#if YAS_TYPE_MAG == YAS_TYPE_M_MAG
	driver.mag.term();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AM_ACC || YAS_TYPE_MAG == YAS_TYPE_AM_MAG
	driver.am.term();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AG_ACC || YAS_TYPE_GYRO == YAS_TYPE_AG_GYRO
	driver.ag.term();
#endif
#if YAS_TYPE_ACC == YAS_TYPE_AMG_ACC || YAS_TYPE_MAG == YAS_TYPE_AMG_MAG \
	|| YAS_TYPE_GYRO == YAS_TYPE_AMG_GYRO
	driver.amg.term();
#endif
#if YAS_LOG_ENABLE
	driver.log_enable = 0;
	driver.log_initialized = 0;
#endif
	driver.initialized = 0;
	return YAS_NO_ERROR;
}