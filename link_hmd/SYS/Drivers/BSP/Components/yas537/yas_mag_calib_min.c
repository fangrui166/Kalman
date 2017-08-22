/*
 * CONFIDENTIAL
 * Copyright (c) 2013-2015 Yamaha Corporation
 */

#include "yas.h"

#if YAS_MAG_CALIB_MINI_ENABLE

#define IS_OUFLOW(a) (((a) % 10) != 0)

#define VAL2COUNT(a)		((a) / 400)
#define COUNT2VAL(a)		((a) * 400)
#define SAMPLE_UNIT		(16)
#define SAMPLE_LEVEL		(3)
#define MAX_SAMPLES		(SAMPLE_UNIT*SAMPLE_LEVEL)
#define MSSOLVER_SIZE		(5)
#define DISTANCE_THRESHOLD	(10)
#define THRESHOLD_2D		(1000)
#define MSJACOBI_LOOP		(6)
#define square(a)		((a) * (a))
#define square_distance(a, b) (square((a)[0]-(b)[0]) + \
		square((a)[1]-(b)[1]) + square((a)[2]-(b)[2]))

struct jacobi_eigen_pair {
	int32_t value;
	struct yas_vector vec;
};
struct jacobi_eigen_pairset {
	struct jacobi_eigen_pair eigenpair[3];
};
struct jacobi_symmetric_matrix33 {
	int32_t val[6];
};
struct solver_symmetric_matrix {
	int32_t val[15];
};
struct solver_vector {
	int32_t val[MSSOLVER_SIZE];
};
struct jacobi_matrix33 {
	int32_t val[3][3];
};
struct mag_calib {
	int initialized;
	int16_t samples[MAX_SAMPLES][3];
	struct yas_mag_calib_config config;
	struct yas_mag_calib_result result;
	struct yas_vector offset;
	int8_t num;
	int8_t index;
};

static const struct yas_vector vector0 = { {0, 0, 0} };
static const uint8_t default_variation[] = {30, 20, 20};
static const uint16_t default_spread[] = {200, 300, 500};
static struct mag_calib module;

static const uint8_t min_sqrttable[] = {
	0, 16, 22, 27, 32, 35, 39, 42, 45, 48, 50, 53, 55, 57,
	59, 61, 64, 65, 67, 69, 71, 73, 75, 76, 78, 80, 81, 83,
	84, 86, 87, 89, 90, 91, 93, 94, 96, 97, 98, 99, 101, 102,
	103, 104, 106, 107, 108, 109, 110, 112, 113, 114, 115, 116, 117, 118,
	119, 120, 121, 122, 123, 124, 125, 126, 128, 128, 129, 130, 131, 132,
	133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 144, 145,
	146, 147, 148, 149, 150, 150, 151, 152, 153, 154, 155, 155, 156, 157,
	158, 159, 160, 160, 161, 162, 163, 163, 164, 165, 166, 167, 167, 168,
	169, 170, 170, 171, 172, 173, 173, 174, 175, 176, 176, 177, 178, 178,
	179, 180, 181, 181, 182, 183, 183, 184, 185, 185, 186, 187, 187, 188,
	189, 189, 190, 191, 192, 192, 193, 193, 194, 195, 195, 196, 197, 197,
	198, 199, 199, 200, 201, 201, 202, 203, 203, 204, 204, 205, 206, 206,
	207, 208, 208, 209, 209, 210, 211, 211, 212, 212, 213, 214, 214, 215,
	215, 216, 217, 217, 218, 218, 219, 219, 220, 221, 221, 222, 222, 223,
	224, 224, 225, 225, 226, 226, 227, 227, 228, 229, 229, 230, 230, 231,
	231, 232, 232, 233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238,
	239, 240, 240, 241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246,
	246, 247, 247, 248, 248, 249, 249, 250, 250, 251, 251, 252, 252, 253,
	253, 254, 254, 255
};

static int32_t sqrt32(int32_t x)
{
	int32_t xn;
	if (x >= 0x10000) {
		if (x >= 0x1000000) {
			if (x >= 0x10000000) {
				if (x >= 0x40000000) {
					if (x >= 0x7ffea810)
						return 46340;
					xn = min_sqrttable[x >> 24] << 8;
				} else
					xn = min_sqrttable[x >> 22] << 7;
			} else {
				if (x >= 0x4000000)
					xn = min_sqrttable[x >> 20] << 6;
				else
					xn = min_sqrttable[x >> 18] << 5;
			}
			xn = (xn + 1 + (x / xn)) >> 1;
			xn = (xn + 1 + (x / xn)) >> 1;
			return ((xn * xn) > x) ? --xn : xn;
		} else {
			if (x >= 0x100000) {
				if (x >= 0x400000)
					xn = min_sqrttable[x >> 16] << 4;
				else
					xn = min_sqrttable[x >> 14] << 3;
			} else {
				if (x >= 0x40000)
					xn = min_sqrttable[x >> 12] << 2;
				else
					xn = min_sqrttable[x >> 10] << 1;
			}
			xn = (xn + 1 + (x / xn)) >> 1;
			return ((xn * xn) > x) ? --xn : xn;
		}
	} else {
		if (x >= 0x100) {
			if (x >= 0x1000) {
				if (x >= 0x4000)
					xn = (min_sqrttable[x >> 8]) + 1;
				else
					xn = (min_sqrttable[x >> 6] >> 1) + 1;
			} else {
				if (x >= 0x400)
					xn = (min_sqrttable[x >> 4] >> 2) + 1;
				else
					xn = (min_sqrttable[x >> 2] >> 3) + 1;
			}
			return ((xn * xn) > x) ? --xn : xn;
		} else {
			if (x >= 0)
				return min_sqrttable[x] >> 4;
			else
				return -1;
		}
	}
}

static int calib_offset_differ(struct yas_mag_calib_result *p1,
		struct yas_mag_calib_result *p2)
{
	int i;
	for (i = 0; i < 3; i++)
		if (p1->offset.v[i] != p2->offset.v[i])
			return 1;
	if (p1->accuracy != p2->accuracy)
		return 1;
	return 0;
}

static int solve(const struct solver_symmetric_matrix *matrix5,
		const struct solver_vector *vector5,
		struct solver_vector *ans5)
{
	int32_t gmatrix[MSSOLVER_SIZE][MSSOLVER_SIZE];
	int32_t tmp;
	int32_t max;
	int i, j, k, pivot;
	*ans5 = *vector5;
	gmatrix[0][0]                 = matrix5->val[0];
	gmatrix[0][1] = gmatrix[1][0] = matrix5->val[1];
	gmatrix[0][2] = gmatrix[2][0] = matrix5->val[2];
	gmatrix[0][3] = gmatrix[3][0] = matrix5->val[3];
	gmatrix[0][4] = gmatrix[4][0] = matrix5->val[4];
	gmatrix[1][1]                 = matrix5->val[5];
	gmatrix[1][2] = gmatrix[2][1] = matrix5->val[6];
	gmatrix[1][3] = gmatrix[3][1] = matrix5->val[7];
	gmatrix[1][4] = gmatrix[4][1] = matrix5->val[8];
	gmatrix[2][2]                 = matrix5->val[9];
	gmatrix[2][3] = gmatrix[3][2] = matrix5->val[10];
	gmatrix[2][4] = gmatrix[4][2] = matrix5->val[11];
	gmatrix[3][3]                 = matrix5->val[12];
	gmatrix[3][4] = gmatrix[4][3] = matrix5->val[13];
	gmatrix[4][4]                 = matrix5->val[14];
	for (i = 0; i < MSSOLVER_SIZE; ++i) {
		max = gmatrix[i][i];
		pivot = i;
		for (j = i + 1; j < MSSOLVER_SIZE; ++j) {
			if (ABS(max) < ABS(gmatrix[j][i])) {
				pivot = j;
				max = gmatrix[j][i];
			}
		}
		if (pivot != i) {
			for (j = 0; j < MSSOLVER_SIZE; ++j) {
				tmp = gmatrix[i][j];
				gmatrix[i][j] = gmatrix[pivot][j];
				gmatrix[pivot][j] = tmp;
			}
			tmp = ans5->val[i];
			ans5->val[i] = ans5->val[pivot];
			ans5->val[pivot] = tmp;
		}
		for (j = i + 1; j < MSSOLVER_SIZE; ++j)
			/* diagonal elements of gmatrix can never be zero. */
			if (gmatrix[i][i] != 0) {
				gmatrix[i][j] <<= 6;
				if ((gmatrix[i][i]>>6) == 0)
					return -1;
				gmatrix[i][j] /= gmatrix[i][i]>>6;
			}
		if (gmatrix[i][i] == 0)
			/* diagonal elements of gmatrix can never be zero. */
			return -1;
		ans5->val[i] /= gmatrix[i][i];
		for (k = i + 1; k < MSSOLVER_SIZE; ++k) {
			for (j = i + 1; j < MSSOLVER_SIZE; ++j)
				gmatrix[k][j] -= ((gmatrix[i][j]>>3)
						* (gmatrix[k][i]>>3))>>6;
			ans5->val[k] -= ans5->val[i] * gmatrix[k][i];
		}
	}
	for (i = MSSOLVER_SIZE - 1; i >= 0; i--)
		for (j = i + 1; j < MSSOLVER_SIZE; ++j)
			ans5->val[i] -= (gmatrix[i][j] * ans5->val[j])>>12;
	return 0;
}

static int jacobi_sort(const struct jacobi_symmetric_matrix33 *valm,
		const struct jacobi_matrix33 *vecm,
		struct jacobi_eigen_pairset *pair)
{
	static const int8_t msjacobi_idx_table[3] = {0, 3, 5};
	int32_t max = 0, min = 0;
	int max_index = 0, min_index = 0, used[3] = {0}, sorted_idx[3] = {0};
	int i, j;
	for (i = 0; i < 3; ++i) {
		if (i == 0) {
			max = min = valm->val[msjacobi_idx_table[i]];
			max_index = min_index = i;
		} else {
			if (max <= valm->val[msjacobi_idx_table[i]]) {
				max = valm->val[msjacobi_idx_table[i]];
				max_index = i;
			} else if (valm->val[msjacobi_idx_table[i]] <= min) {
				min = valm->val[msjacobi_idx_table[i]];
				min_index = i;
			}
		}
	}
	used[max_index] = used[min_index] = 1;
	for (i = 0; i < 3; ++i) {
		/* break in i == 0 or 1 or 2. */
		if (used[i] == 0) {
			sorted_idx[1] = i;
			break;
		}
	}
	sorted_idx[0] = max_index;
	sorted_idx[2] = min_index;
	for (i = 0; i < 3; ++i) {
		pair->eigenpair[i].value
			= valm->val[msjacobi_idx_table[sorted_idx[i]]];
#ifndef MSJACOBI_DISABLE_NONNEGATIVE_LIMITATION
		if (pair->eigenpair[i].value <= 0)
			pair->eigenpair[i].value = 0;
#endif
		for (j = 0; j < 3; ++j)
			pair->eigenpair[i].vec.v[j]
				= vecm->val[j][sorted_idx[i]];
	}
	return 0;
}

static int jacobi_calc_eigenpair(const struct jacobi_symmetric_matrix33
		*matrix3, struct jacobi_eigen_pairset *pair)
{
	static const uint8_t jacobi_table[3][3] = {
		{ 0, 1, 2}, { 1, 3, 4}, { 2, 4, 5}
	};
	struct jacobi_symmetric_matrix33 valm;
	struct jacobi_symmetric_matrix33 nextmval;
	struct jacobi_matrix33 vecm;
	int32_t v1, v2, v3, msfc2, msfs2, msfc, msfs, msfcs, alpha, beta;
	int i, j, k, l, m, max_idx;
	valm = *matrix3;
	for (i = 0; i < 3; ++i)
		for (j = 0; j < 3; ++j)
			if (i == j)
				vecm.val[i][j] = 0x4000;
			else
				vecm.val[i][j] = 0;
	for (i = 0; i < MSJACOBI_LOOP; ++i) {
		max_idx = 1; k = 0; l = 1; m = 2;
		if (ABS(valm.val[1]) < ABS(valm.val[2])) {
			max_idx = 2;
			l = 2;
			m = 1;
		}
		if (ABS(valm.val[max_idx]) < ABS(valm.val[4])) {
			k = 1;
			l = 2;
			m = 0;
		}
		alpha = (valm.val[jacobi_table[l][l]]
				- valm.val[jacobi_table[k][k]])>>1;
		beta = sqrt32(square(alpha/10)
				+ square(valm.val[jacobi_table[k][l]]/10))*10;
		if (beta == 0)
			continue;
		msfc2 = ((1<<12) + (ABS(alpha)<<12) / beta) >> 1;
		msfc = sqrt32(msfc2);
		if ((beta*msfc>>2) == 0)
			/* never be beta*msfc>>2 == 0. */
			return -1;
		msfs = (valm.val[jacobi_table[k][l]]<<12)
			/ ((beta * msfc)>>2);
		if (valm.val[jacobi_table[l][l]] < valm.val[jacobi_table[k][k]])
			msfs = -msfs;
		msfs2 = (msfs * msfs)>>6;
		msfcs = (msfs * msfc)>>6;
		nextmval = valm;
		v1 = (valm.val[jacobi_table[k][m]] * msfc)>>6;
		v2 = (valm.val[jacobi_table[l][m]] * msfs)>>9;
		nextmval.val[jacobi_table[k][m]] = v1 - v2;
		v1 = (valm.val[jacobi_table[k][m]] * msfs)>>9;
		v2 = (valm.val[jacobi_table[l][m]] * msfc)>>6;
		nextmval.val[jacobi_table[l][m]] = v1 + v2;
		v1 = (valm.val[jacobi_table[k][l]] * msfcs)>>8;
		v2 = (valm.val[jacobi_table[k][k]] * msfc2)>>12;
		v3 = (valm.val[jacobi_table[l][l]] * msfs2)>>12;
		nextmval.val[jacobi_table[k][k]] = -v1 + v2 + v3;
		v2 = (valm.val[jacobi_table[l][l]] * msfc2)>>12;
		v3 = (valm.val[jacobi_table[k][k]] * msfs2)>>12;
		nextmval.val[jacobi_table[l][l]] = v1 + v2 + v3;
		nextmval.val[jacobi_table[k][l]] = 0;
		for (j = 0; j < 3; ++j) {
			v1 = vecm.val[j][k];
			v2 = vecm.val[j][l];
			vecm.val[j][k] = (v1 * msfc - ((v2 * msfs)>>3))>>6;
			vecm.val[j][l] = (v1 * (msfs>>3) + v2 * msfc)>>6;
		}
		valm = nextmval;
	}
	jacobi_sort(&valm, &vecm, pair);
	return 0;
}

static int data_update(int16_t *mag)
{
	int i;
	if (0 < module.num) {
		int32_t sqdir = 0;
		int idx;
		idx = module.index - 1;
		if (idx < 0)
			idx += MAX_SAMPLES;
		sqdir = square_distance(module.samples[idx], mag);
		if (sqdir < square(DISTANCE_THRESHOLD))
			return 0;
	}
	for (i = 0; i < 3; i++)
		module.samples[module.index][i] = mag[i];
	if (MAX_SAMPLES <= ++module.index)
		module.index = 0;
	if (MAX_SAMPLES <= ++module.num)
		module.num = MAX_SAMPLES;
	return module.num%SAMPLE_UNIT == 0 && module.index%SAMPLE_UNIT == 0;
}

static void data_notify_high_distortion(void)
{
	if (module.num == SAMPLE_UNIT)
		module.num = 0;
	else
		module.num = SAMPLE_UNIT;
}

static int32_t radius_variance(struct yas_vector *center, int32_t *radius)
{
	int32_t sq, sum = 0, avg = 0;
	int16_t r[MAX_SAMPLES];
	int i, idx;
	idx = module.index - module.num;
	if (idx < 0)
		idx += MAX_SAMPLES;
	for (i = 0; i < module.num; i++) {
		sq = square_distance(center->v, module.samples[idx]);
		r[i] = (int16_t) sqrt32(sq);
		sum += r[i];
		if (++idx == MAX_SAMPLES)
			idx = 0;
	}
	avg = sum / module.num;
	*radius = avg;
	sum = 0;
	for (i = 0; i < module.num; i++) {
		sq = square(r[i] - avg);
		sum += sq;
	}
	return sum / module.num;
}

static int16_t check_resemblance(struct yas_vector *p0, struct yas_vector *p1,
		struct yas_vector *centroid, struct yas_vector *paxis)
{
	struct yas_vector center = *p1;
	int32_t k = 0, ip = 0, norm2 = 0, tmp;
	int i;
	for (i = 0; i < 3; ++i)
		k += paxis->v[i] * (centroid->v[i] - p1->v[i]);
	k /= 0x4000;
	for (i = 0; i < 3; ++i)
		center.v[i] += k * paxis->v[i] / 0x4000;
	for (i = 0; i < 3; ++i) {
		tmp = p0->v[i] - center.v[i];
		ip += paxis->v[i] * tmp;
		norm2 += tmp * tmp;
	}
	if (norm2 == 0)
		return 0x101;
	if (ip < 0)
		ip = -ip;
	ip >>= 10;
	return (int16_t)((ip * ip) / norm2);
}

static int calc_offset(int16_t *c, struct yas_vector *current,
		struct yas_vector *centroid,
		struct jacobi_symmetric_matrix33 *matrix3,
		struct jacobi_eigen_pairset *pair, struct yas_vector *vector3,
		struct yas_vector *offset)
{
	static const int8_t idx_table_1[6] = {0, 1, 2, 5, 6, 9};
	static const int8_t idx_table_2[2][3] = {
		{3, 7, 10}, { 4, 8, 11}
	};
	struct solver_symmetric_matrix matrix5;
	struct solver_vector vector5, ans5;
	int32_t temp_s32;
	int i, j, idx;
	for (i = 0; i < 15; i++)
		matrix5.val[i] = 0;
	for (i = 0; i < MSSOLVER_SIZE; i++)
		vector5.val[i] = ans5.val[i] = 0;
	for (i = 0; i < 6; ++i)
		matrix5.val[idx_table_1[i]] = matrix3->val[i];
	for (i = 0; i < 2; ++i) {
		for (j = 0; j < 3; ++j) {
			idx = idx_table_2[i][j];
			matrix5.val[idx] = c[i]
				* pair->eigenpair[i + 1].vec.v[j];
			if (matrix5.val[idx] != 0)
				matrix5.val[idx] >>= 10;
		}
	}
	for (i = 0; i < 3; ++i)
		vector5.val[i] = vector3->v[i];
	for (i = 0, idx = 12; i < 2; ++i, idx += 2) {
		if (pair->eigenpair[0].value == 0)
			/* greatest eigenvalue can never be zero. */
			return -1;
		matrix5.val[idx] = ((c[i] - (1<<10))<<18)
			/ pair->eigenpair[0].value;
		temp_s32 = 0;
		for (j = 0; j < 3; ++j)
			temp_s32 += (int32_t)pair->eigenpair[i + 1].vec.v[j]
				* (current->v[j] - centroid->v[j]);
		vector5.val[i + 3] = (temp_s32>>10) * c[i];
	}
	solve(&matrix5, &vector5, &ans5);
	for (i = 0; i < 3; ++i) {
		offset->v[i] = ans5.val[i];
		offset->v[i] += centroid->v[i];
	}
	return 0;
}

static int create_equation(struct yas_vector *centroid,
		struct jacobi_symmetric_matrix33 *simultaneous_l,
		struct yas_vector *simultaneous_r)
{
	struct yas_vector v;
	int32_t R = 0, tmp;
	int i, j, k, idx, idx2;
	*centroid = vector0;
	for (i = 0; i < 6; ++i)
		simultaneous_l->val[i] = 0;
	for (i = 0; i < 3; ++i)
		simultaneous_r->v[i] = 0;
	idx = module.index - module.num;
	if (idx < 0)
		idx += MAX_SAMPLES;
	for (i = 0; i < module.num; ++i) {
		for (j = 0; j < 3; ++j)
			centroid->v[j] += module.samples[idx][j];
		if (++idx == MAX_SAMPLES)
			idx = 0;
	}
	for (i = 0; i < 3; ++i)
		centroid->v[i] /= module.num;
	idx = module.index - module.num;
	if (idx < 0)
		idx += MAX_SAMPLES;
	for (i = 0; i < module.num; ++i) {
		for (j = 0; j < 3; ++j)
			R += square(module.samples[idx][j] - centroid->v[j]);
		if (++idx == MAX_SAMPLES)
			idx = 0;
	}
	R /= module.num;
	idx = module.index - module.num;
	if (idx < 0)
		idx += MAX_SAMPLES;
	for (i = 0; i < module.num; ++i) {
		for (j = 0; j < 3; ++j)
			v.v[j] = module.samples[idx][j] - centroid->v[j];
		if (++idx == MAX_SAMPLES)
			idx = 0;
		for (j = idx2 = 0; j < 3; ++j) {
			for (k = j; k < 3; k++, idx2++)
				simultaneous_l->val[idx2] += v.v[j] * v.v[k];
		}
		tmp = 0;
		for (j = 0; j < 3; ++j)
			tmp += square(v.v[j]);
		for (j = 0; j < 3; ++j)
			simultaneous_r->v[j] += (v.v[j] * (tmp - R)) >> 1;
	}
	return 0;
}

static int algo_update(struct yas_mag_calib_result *r)
{
	struct jacobi_symmetric_matrix33 matrix3;
	struct jacobi_eigen_pairset pairset;
	struct yas_vector centroid, vector3, offset;
	int32_t radius = 0, spread_3d, spread_2d, variation = 0, max_ip;
	int32_t ip;
	int16_t resemblance;
	uint8_t axis = 0, accuracy = 0;
	int16_t constraint[2] = {0, 0};
	int i, idx, rt;
	if (create_equation(&centroid, &matrix3, &vector3) < 0)
		/* create_equation() never return an error. */
		return 0;
	rt = jacobi_calc_eigenpair(&matrix3, &pairset);
	if (rt < 0)
		/* jacobi_calc_eigenpair() never return an error. */
		return 0;
	constraint[1] = 1;
	if (calc_offset(constraint, &module.offset, &centroid, &matrix3,
				&pairset, &vector3, &offset) < 0)
		/* calc_offset() never return an error. */
		return 0;
	variation = radius_variance(&offset, &radius);
	if (radius <= 25) /* 10[uT] */
		return 0;
	if (radius <= 100)
		variation = variation * 100 / radius;
	spread_2d = pairset.eigenpair[1].value / module.num*10000
		/ radius / radius;
	spread_3d = pairset.eigenpair[2].value / module.num*10000
		/ radius / radius;
	r->spread = (uint16_t) spread_3d;
	r->variation = (uint16_t) variation;
	r->radius = (uint16_t)COUNT2VAL(radius);
	r->axis = 0;
	r->level = (uint8_t) module.num;
	accuracy = 0;
	for (i = 2; 0 <= i; i--) {
		idx = SAMPLE_LEVEL - module.num / SAMPLE_UNIT;
		if (module.config.spread[i] + idx * 200 < spread_3d) {
			accuracy = (uint8_t)(i + 1);
			break;
		}
	}
	if (0 < accuracy) {
		for (i = accuracy - 1; 0 <= i; i--) {
			if (variation <= module.config.variation[i]
					+ (spread_3d / 250 >= 5 ? 5
						: spread_3d / 250)) {
				accuracy = (uint8_t)(i + 1);
				break;
			}
		}
		if (i < 0)
			/* never be i < 0. */
			return -3;
		r->axis = 0x07;
		if (r->accuracy <= accuracy) {
			module.offset = offset;
			r->accuracy = accuracy;
			for (i = 0; i < 3; i++)
				r->offset.v[i] = COUNT2VAL(offset.v[i]);
			return 1;
		}
	}
	constraint[1] = 1<<10;
	if (calc_offset(constraint, &module.offset, &centroid, &matrix3,
				&pairset, &vector3, &offset) < 0)
		/* calc_offset() never return an error. */
		return 0;
	variation = radius_variance(&offset, &radius);
	if (radius <= 0)
		/* never be radius <= 0. */
		return 0;
	r->variation = (uint16_t)variation;
	r->radius = (uint16_t)COUNT2VAL(radius);
	if (THRESHOLD_2D < spread_2d) {
		if (r->accuracy == 0)
			idx = 0;
		else
			idx = r->accuracy - 1;
		if (module.config.variation[idx] / 2 + spread_2d / 400
				< variation)
			return -3;
		resemblance = check_resemblance(&module.offset, &offset,
				&centroid, &pairset.eigenpair[2].vec);
		idx = 0;
		max_ip = -1;
		for (i = 0; i < 3; ++i) {
			ip = ABS(pairset.eigenpair[2].vec.v[i]);
			if (max_ip < ip) {
				max_ip = ip;
				idx = i;
			}
		}
		if (idx == 0)
			axis = 0x06;
		if (idx == 1)
			axis = 0x05;
		if (idx == 2)
			axis = 0x03;
		r->axis = axis;
		if (resemblance < 231) {
			module.offset = offset;
			for (i = 0; i < 3; i++)
				r->offset.v[i] = COUNT2VAL(offset.v[i]);
		}
		return 1;
	}
	return 0;
}

static int yas_get_offset(int type, struct yas_vector *calib_offset,
		uint8_t *accuracy)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (type != YAS_TYPE_MAG)
		return YAS_ERROR_ARG;
	if (calib_offset == NULL || accuracy == NULL)
		return YAS_ERROR_ARG;
	*calib_offset = module.result.offset;
	*accuracy = module.result.accuracy;
	return YAS_NO_ERROR;
}

static int yas_set_offset(int type, struct yas_vector *calib_offset,
		uint8_t accuracy)
{
	int i;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (type != YAS_TYPE_MAG)
		return YAS_ERROR_ARG;
	if (calib_offset == NULL)
		return YAS_ERROR_ARG;
	if (3 < accuracy)
		return YAS_ERROR_ARG;
	module.result.offset = *calib_offset;
	for (i = 0; i < 3; i++)
		module.offset.v[i] = VAL2COUNT(calib_offset->v[i]);
	module.result.accuracy = (uint8_t) accuracy;
	return YAS_NO_ERROR;
}

static int yas_update(struct yas_data *mag, int num)
{
	struct yas_mag_calib_result prev = module.result;
	int16_t data[3];
	int i, rt, j;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (mag == NULL || num < 0)
		return YAS_ERROR_ARG;
	if (num == 0)
		return YAS_NO_ERROR;
	for (i = 0; i < num; i++) {
		if (mag[i].type != YAS_TYPE_MAG)
			continue;
		if (IS_OUFLOW(mag[i].xyz.v[0])
				|| IS_OUFLOW(mag[i].xyz.v[1])
				|| IS_OUFLOW(mag[i].xyz.v[2]))
			module.result.accuracy = 0;
		for (j = 0; j < 3; j++)
			data[j] = (int16_t)VAL2COUNT(mag[i].xyz.v[j]);
		rt = 0;
		if (data_update(data))
			rt = algo_update(&module.result);
		if (rt < 0) {
			if (rt == -3)
				data_notify_high_distortion();
			else
				/* the error code of algo_update is only -3. */
				return rt;
		}
	}
	if (calib_offset_differ(&prev, &module.result))
		return 1;
	return 0;
}

static int yas_init(void)
{
	int i;
	if (module.initialized)
		return YAS_ERROR_INITIALIZE;
	module.num = 0;
	module.index = 0;
	module.initialized = 1;
	module.config.mode = YAS_MAG_CALIB_MODE_SPHERE;
	for (i = 0; i < 3; i++)
		module.config.spread[i] = default_spread[i];
	for (i = 0; i < 3; i++)
		module.config.variation[i] = default_variation[i];
	module.offset = vector0;
	module.result.offset = vector0;
	module.result.accuracy = 0;
	module.result.spread = 0;
	module.result.variation = 0;
	module.result.radius = 0;
	module.result.axis = 0;
	module.result.level = 0;
	return YAS_NO_ERROR;
}

static int yas_term(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	module.initialized = 0;
	return YAS_NO_ERROR;
}

static int yas_get_config(struct yas_mag_calib_config *c)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (c == NULL)
		return YAS_ERROR_ARG;
	*c = module.config;
	return 0;
}

static int yas_set_config(struct yas_mag_calib_config *c)
{
	struct yas_vector calib_offset;
	uint8_t accuracy;
	int i;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (c == NULL)
		return YAS_ERROR_ARG;
	if (c->mode != YAS_MAG_CALIB_MODE_SPHERE)
		return YAS_ERROR_ARG;
	for (i = 0; i < 3; i++)
		if (10000 < c->spread[i])
			return YAS_ERROR_ARG;
	yas_get_offset(YAS_TYPE_MAG, &calib_offset, &accuracy);
	yas_term();
	yas_init();
	yas_set_offset(YAS_TYPE_MAG, &calib_offset, accuracy);
	module.config = *c;
	return YAS_NO_ERROR;
}

static int yas_get_result(struct yas_mag_calib_result *r)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (r == NULL)
		return YAS_ERROR_ARG;
	*r = module.result;
	return YAS_NO_ERROR;
}

static int yas_reset(void)
{
	struct yas_mag_calib_config config;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	config = module.config;
	yas_term();
	yas_init();
	yas_set_config(&config);
	return YAS_NO_ERROR;
}

int yas_mag_calib_init(struct yas_mag_calib *f)
{
	if (f == NULL)
		return YAS_ERROR_ARG;
	f->init = yas_init;
	f->term = yas_term;
	f->reset = yas_reset;
	f->update = yas_update;
	f->get_offset = yas_get_offset;
	f->set_offset = yas_set_offset;
	f->get_config = yas_get_config;
	f->set_config = yas_set_config;
	f->get_result = yas_get_result;
	module.initialized = 0;
	return YAS_NO_ERROR;
}
#endif
