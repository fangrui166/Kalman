#ifndef __FUSION_H__
#define __FUSION_H__
#include "vec.h"
#include "mat.h"
#include "quat.h"

#include <stdint.h>

#define EIO         5
#define ENXIO       6
#define ENOMEM      12
#define EBUSY       16
#define ENODEV      19
#define EINVAL      22
#define EOPNOTSUPP  95

struct FusionParam {
  float gyro_var;
  float gyro_bias_var;
  float acc_stdev;
  float mag_stdev;
};

struct Fusion {
    Quat x0;
    struct Vec3 x1;

    struct Mat33 P[2][2];
    struct Mat33 GQGt[2][2];

    struct Mat33 Phi0[2];
    struct Vec3 Ba, Bm;
    uint32_t mInitState;
    float mPredictDt;
    struct Vec3 mData[3];
    uint32_t mCount[3];
    uint32_t flags;

    float  fake_mag_decimation;
    struct FusionParam param;
};

enum FusionFlagBits {
    FUSION_USE_MAG      = 1 << 0,
    FUSION_USE_GYRO     = 1 << 1,
    FUSION_REINITIALIZE = 1 << 2,
};

typedef enum{
    ACC_T=0,
    MAG_T,
    GYRO_T,
}SENSOR_TYPE;
void fusionGetRotationMatrix(const struct Fusion *fusion, struct Mat33 *R);
int on_sensor_changed(uint8_t type, float *vec, long long dT);
int fusion_init(void);
int get_quaternion(float quaternion[]);

#endif
