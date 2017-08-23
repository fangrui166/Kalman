#include "cmsis_os.h"
#include "fusion.h"
#include <stdio.h>
#include "mymath.h"
#include <math.h>
#include "arm_math.h"

#define ACC     1
#define MAG     2
#define GYRO    4

#define DEFAULT_GYRO_VAR         1e-7f
#define DEFAULT_GYRO_BIAS_VAR    1e-12f
#define DEFAULT_ACC_STDEV        5e-2f
#define DEFAULT_MAG_STDEV        5e-1f

#define GEOMAG_GYRO_VAR          2e-4f
#define GEOMAG_GYRO_BIAS_VAR     1e-4f
#define GEOMAG_ACC_STDEV         0.02f
#define GEOMAG_MAG_STDEV         0.02f

#define SYMMETRY_TOLERANCE       1e-10f
#define FAKE_MAG_INTERVAL        1.0f  //sec

#define NOMINAL_GRAVITY          9.81f
#define FREE_FALL_THRESHOLD      (0.1f * NOMINAL_GRAVITY)
#define FREE_FALL_THRESHOLD_SQ   (FREE_FALL_THRESHOLD * FREE_FALL_THRESHOLD)

#define MAX_VALID_MAGNETIC_FIELD    75.0f
#define MAX_VALID_MAGNETIC_FIELD_SQ (MAX_VALID_MAGNETIC_FIELD *\
                                    MAX_VALID_MAGNETIC_FIELD)

#define MIN_VALID_MAGNETIC_FIELD    30.0f
#define MIN_VALID_MAGNETIC_FIELD_SQ (MIN_VALID_MAGNETIC_FIELD *\
                                    MIN_VALID_MAGNETIC_FIELD)

#define MIN_VALID_CROSS_PRODUCT_MAG     1.0e-3
#define MIN_VALID_CROSS_PRODUCT_MAG_SQ  (MIN_VALID_CROSS_PRODUCT_MAG *\
                                        MIN_VALID_CROSS_PRODUCT_MAG)

#define DELTA_TIME_MARGIN 1.0e-9f

static struct Fusion fusion_data = {0};
static osThreadId fusion_thread_handle;
static osMessageQId fusion_MsgQueueHandle;
struct sensor_data{
    uint8_t type;
    struct Vec3 vec;
    float dT;

};
struct sensor_data sensor_data;
void initFusion(struct Fusion *fusion, uint32_t flags)
{
    fusion->flags = flags;
    printf("%s \r\n",__func__);
    if (flags & FUSION_USE_GYRO) {
        // normal fusion mode
        fusion->param.gyro_var = DEFAULT_GYRO_VAR;
        fusion->param.gyro_bias_var = DEFAULT_GYRO_BIAS_VAR;
        fusion->param.acc_stdev = DEFAULT_ACC_STDEV;
        fusion->param.mag_stdev = DEFAULT_MAG_STDEV;
    } else {
        // geo mag mode
        fusion->param.gyro_var = GEOMAG_GYRO_VAR;
        fusion->param.gyro_bias_var = GEOMAG_GYRO_BIAS_VAR;
        fusion->param.acc_stdev = GEOMAG_ACC_STDEV;
        fusion->param.mag_stdev = GEOMAG_MAG_STDEV;
    }

    if (flags & FUSION_REINITIALIZE)
    {
        initVec3(&fusion->Ba, 0.0f, 0.0f, 1.0f);
        initVec3(&fusion->Bm, 0.0f, 1.0f, 0.0f);

        initVec4(&fusion->x0, 0.0f, 0.0f, 0.0f, 0.0f);
        initVec3(&fusion->x1, 0.0f, 0.0f, 0.0f);

        fusion->mInitState = 0;

        fusion->mPredictDt = 0.0f;
        fusion->mCount[ACC_T]  = 0;
        fusion->mCount[MAG_T]  = 0;
        fusion->mCount[GYRO_T] = 0;

        initVec3(&fusion->mData[ACC_T],  0.0f, 0.0f, 0.0f);
        initVec3(&fusion->mData[MAG_T],  0.0f, 0.0f, 0.0f);
        initVec3(&fusion->mData[GYRO_T], 0.0f, 0.0f, 0.0f);
    } else  {
        // mask off disabled sensor bit
        fusion->mInitState &= (ACC |
                            ((fusion->flags & FUSION_USE_MAG) ? MAG : 0) |
                            ((fusion->flags & FUSION_USE_GYRO) ? GYRO : 0));
    }
}


int fusionHasEstimate(const struct Fusion *fusion) {
    // waive sensor init depends on the mode
    return fusion->mInitState == (ACC |
                                  ((fusion->flags & FUSION_USE_MAG) ? MAG : 0) |
                                  ((fusion->flags & FUSION_USE_GYRO) ? GYRO : 0)
                                 );
}
static void updateDt(struct Fusion *fusion, float dT)
{
    if (fabsf(fusion->mPredictDt - dT) > DELTA_TIME_MARGIN) {
        float dT2 = dT * dT;
        float dT3 = dT2 * dT;

        float q00 = fusion->param.gyro_var * dT +
                    0.33333f * fusion->param.gyro_bias_var * dT3;
        float q11 = fusion->param.gyro_bias_var * dT;
        float q10 = 0.5f * fusion->param.gyro_bias_var * dT2;
        float q01 = q10;

        initDiagonalMatrix(&fusion->GQGt[0][0], q00);
        initDiagonalMatrix(&fusion->GQGt[0][1], -q10);
        initDiagonalMatrix(&fusion->GQGt[1][0], -q01);
        initDiagonalMatrix(&fusion->GQGt[1][1], q11);
        fusion->mPredictDt = dT;
    }
}

int fusionInitComplete(struct Fusion *fusion, int sensorType,
    const struct Vec3 *vec, float dT)
{
    if (fusionHasEstimate(fusion)) {
        return 1;
    }

    switch(sensorType){
        case ACC:
          {
            struct Vec3 unityD = *vec;
            if (!(fusion->flags & FUSION_USE_GYRO)) {
                updateDt(fusion, dT);
            }
            vec3Normalize(&unityD);

            vec3Add(&fusion->mData[ACC_T], &unityD);
            ++fusion->mCount[ACC_T];

            if (fusion->mCount[ACC_T] == 8) {
                fusion->mInitState |= ACC;
            }
          }
            break;
        case MAG:
          {
            struct Vec3 unityD = *vec;
            vec3Normalize(&unityD);

            vec3Add(&fusion->mData[MAG_T], &unityD);
            ++fusion->mCount[MAG_T];

            fusion->mInitState |= MAG;
          }
            break;
        case GYRO:
            updateDt(fusion, dT);

            struct Vec3 scaledD = *vec;
            vec3ScalarMul(&scaledD, dT);

            vec3Add(&fusion->mData[GYRO_T], &scaledD);
            ++fusion->mCount[GYRO_T];

            fusion->mInitState |= GYRO;
            break;

    }

    if (fusionHasEstimate(fusion)) {
        vec3ScalarMul(&fusion->mData[ACC_T], 1.0f / fusion->mCount[ACC_T]);

        if (fusion->flags & FUSION_USE_MAG) {
            vec3ScalarMul(&fusion->mData[MAG_T], 1.0f / fusion->mCount[MAG_T]);
        }
        else {
            fusion->fake_mag_decimation = 0.f;
        }

        struct Vec3 up = fusion->mData[ACC_T];

        struct Vec3 east;
        if (fusion->flags & FUSION_USE_MAG) {
            vec3Cross(&east, &fusion->mData[MAG_T], &up);
            vec3Normalize(&east);
        }
        else {
            findOrthogonalVector(up.x, up.y, up.z, &east.x, &east.y, &east.z);
        }

        struct Vec3 north;
        vec3Cross(&north, &up, &east);

        struct Mat33 R;
        initMatrixColumns(&R, &east, &north, &up);

        //Quat q;
        //initQuat(&q, &R);

        initQuat(&fusion->x0, &R);
        initVec3(&fusion->x1, 0.0f, 0.0f, 0.0f);
        printf("%s [quat] x:%f, y:%f, z:%f, w:%f\r\n", __func__,fusion->x0.x, fusion->x0.y,
            fusion->x0.z, fusion->x0.w);
        initZeroMatrix(&fusion->P[0][0]);
        initZeroMatrix(&fusion->P[0][1]);
        initZeroMatrix(&fusion->P[1][0]);
        initZeroMatrix(&fusion->P[1][1]);
    }
    return 0;
}
static void matrixCross(struct Mat33 *out, struct Vec3 *p, float diag)
{
    out->elem[0][0] = diag;
    out->elem[1][1] = diag;
    out->elem[2][2] = diag;
    out->elem[1][0] = p->z;
    out->elem[0][1] = -p->z;
    out->elem[2][0] = -p->y;
    out->elem[0][2] = p->y;
    out->elem[2][1] = p->x;
    out->elem[1][2] = -p->x;
}

static void fusionCheckState(struct Fusion *fusion)
{

    if (!mat33IsPositiveSemidefinite(&fusion->P[0][0], SYMMETRY_TOLERANCE)
            || !mat33IsPositiveSemidefinite(
                &fusion->P[1][1], SYMMETRY_TOLERANCE)) {

        initZeroMatrix(&fusion->P[0][0]);
        initZeroMatrix(&fusion->P[0][1]);
        initZeroMatrix(&fusion->P[1][0]);
        initZeroMatrix(&fusion->P[1][1]);
    }
}

#define kEps 1.0E-4f

 static void fusionPredict(struct Fusion *fusion, const struct Vec3 *w)
{
    const float dT = fusion->mPredictDt;

    Quat q = fusion->x0;
    struct Vec3 b = fusion->x1;

    struct Vec3 we = *w;
    vec3Sub(&we, &b);

    struct Mat33 I33;
    initDiagonalMatrix(&I33, 1.0f);

    struct Mat33 I33dT;
    initDiagonalMatrix(&I33dT, dT);

    struct Mat33 wx;
    matrixCross(&wx, &we, 0.0f);

    struct Mat33 wx2;
    mat33Multiply(&wx2, &wx, &wx);

    float norm_we = vec3Norm(&we);

    if (fabsf(norm_we) < kEps) {
        return;
    }

    float lwedT = norm_we * dT;
    float hlwedT = 0.5f * lwedT;
    float ilwe = 1.0f / norm_we;
    float k0 = (1.0f - arm_cosf(lwedT)) * (ilwe * ilwe);
    float k1 = arm_sinf(lwedT);
    float k2 = arm_cosf(hlwedT);

    struct Vec3 psi = we;
    vec3ScalarMul(&psi, arm_sinf(hlwedT) * ilwe);

    struct Vec3 negPsi = psi;
    vec3ScalarMul(&negPsi, -1.0f);

    struct Mat33 O33;
    matrixCross(&O33, &negPsi, k2);

    struct Mat44 O;
    uint32_t i;
    for (i = 0; i < 3; ++i) {
        uint32_t j;
        for (j = 0; j < 3; ++j) {
            O.elem[i][j] = O33.elem[i][j];
        }
    }

    O.elem[3][0] = -psi.x;
    O.elem[3][1] = -psi.y;
    O.elem[3][2] = -psi.z;
    O.elem[3][3] = k2;

    O.elem[0][3] = psi.x;
    O.elem[1][3] = psi.y;
    O.elem[2][3] = psi.z;

    struct Mat33 tmp = wx;
    mat33ScalarMul(&tmp, k1 * ilwe);

    fusion->Phi0[0] = I33;
    mat33Sub(&fusion->Phi0[0], &tmp);

    tmp = wx2;
    mat33ScalarMul(&tmp, k0);

    mat33Add(&fusion->Phi0[0], &tmp);

    tmp = wx;
    mat33ScalarMul(&tmp, k0);
    fusion->Phi0[1] = tmp;

    mat33Sub(&fusion->Phi0[1], &I33dT);

    tmp = wx2;
    mat33ScalarMul(&tmp, ilwe * ilwe * ilwe * (lwedT - k1));

    mat33Sub(&fusion->Phi0[1], &tmp);

    mat44Apply(&fusion->x0, &O, &q);

    if (fusion->x0.w < 0.0f) {
        fusion->x0.x = -fusion->x0.x;
        fusion->x0.y = -fusion->x0.y;
        fusion->x0.z = -fusion->x0.z;
        fusion->x0.w = -fusion->x0.w;
    }

    // Pnew = Phi * P

    struct Mat33 Pnew[2][2];
    mat33Multiply(&Pnew[0][0], &fusion->Phi0[0], &fusion->P[0][0]);
    mat33Multiply(&tmp, &fusion->Phi0[1], &fusion->P[1][0]);
    mat33Add(&Pnew[0][0], &tmp);

    mat33Multiply(&Pnew[0][1], &fusion->Phi0[0], &fusion->P[0][1]);
    mat33Multiply(&tmp, &fusion->Phi0[1], &fusion->P[1][1]);
    mat33Add(&Pnew[0][1], &tmp);

    Pnew[1][0] = fusion->P[1][0];
    Pnew[1][1] = fusion->P[1][1];

    // P = Pnew * Phi^T

    mat33MultiplyTransposed2(&fusion->P[0][0], &Pnew[0][0], &fusion->Phi0[0]);
    mat33MultiplyTransposed2(&tmp, &Pnew[0][1], &fusion->Phi0[1]);
    mat33Add(&fusion->P[0][0], &tmp);

    fusion->P[0][1] = Pnew[0][1];

    mat33MultiplyTransposed2(&fusion->P[1][0], &Pnew[1][0], &fusion->Phi0[0]);
    mat33MultiplyTransposed2(&tmp, &Pnew[1][1], &fusion->Phi0[1]);
    mat33Add(&fusion->P[1][0], &tmp);

    fusion->P[1][1] = Pnew[1][1];

    mat33Add(&fusion->P[0][0], &fusion->GQGt[0][0]);
    mat33Add(&fusion->P[0][1], &fusion->GQGt[0][1]);
    mat33Add(&fusion->P[1][0], &fusion->GQGt[1][0]);
    mat33Add(&fusion->P[1][1], &fusion->GQGt[1][1]);

    fusionCheckState(fusion);
}

int fusionHandleGyro(struct Fusion *fusion, int sensorType,
    const struct Vec3 *gyro, float dT)
{
    if(!fusionInitComplete(fusion, sensorType, gyro, dT)){
        return -1;
    }
    updateDt(fusion, dT);

    fusionPredict(fusion, gyro);

    return 0;
}
static void scaleCovariance(struct Mat33 *out, const struct Mat33 *A,
    const struct Mat33 *P)
{
    uint32_t r;
    for (r = 0; r < 3; ++r) {
        uint32_t j;
        for (j = r; j < 3; ++j) {
            float apat = 0.0f;
            uint32_t c;
            for (c = 0; c < 3; ++c) {
                float v = A->elem[c][r] * P->elem[c][c] * 0.5f;
                uint32_t k;
                for (k = c + 1; k < 3; ++k) {
                    v += A->elem[k][r] * P->elem[c][k];
                }

                apat += 2.0f * v * A->elem[c][j];
            }

            out->elem[r][j] = apat;
            out->elem[j][r] = apat;
        }
    }
}

static void getF(struct Vec4 F[3], const struct Vec4 *q)
{
    F[0].x = q->w;      F[1].x = -q->z;         F[2].x = q->y;
    F[0].y = q->z;      F[1].y = q->w;          F[2].y = -q->x;
    F[0].z = -q->y;     F[1].z = q->x;          F[2].z = q->w;
    F[0].w = -q->x;     F[1].w = -q->y;         F[2].w = -q->z;
}

static void fusionUpdate( struct Fusion *fusion, const struct Vec3 *z,
                            const struct Vec3 *Bi, float sigma)
{
    struct Mat33 A;
    quatToMatrix(&A, &fusion->x0);

    struct Vec3 Bb;
    mat33Apply(&Bb, &A, Bi);

    struct Mat33 L;
    matrixCross(&L, &Bb, 0.0f);

    struct Mat33 R;
    initDiagonalMatrix(&R, sigma * sigma);

    struct Mat33 S;
    scaleCovariance(&S, &L, &fusion->P[0][0]);

    mat33Add(&S, &R);

    struct Mat33 Si;
    mat33Invert(&Si, &S);

    struct Mat33 LtSi;
    mat33MultiplyTransposed(&LtSi, &L, &Si);

    struct Mat33 K[2];
    mat33Multiply(&K[0], &fusion->P[0][0], &LtSi);
    mat33MultiplyTransposed(&K[1], &fusion->P[0][1], &LtSi);

    struct Mat33 K0L;
    mat33Multiply(&K0L, &K[0], &L);

    struct Mat33 K1L;
    mat33Multiply(&K1L, &K[1], &L);

    struct Mat33 tmp;
    mat33Multiply(&tmp, &K0L, &fusion->P[0][0]);
    mat33Sub(&fusion->P[0][0], &tmp);

    mat33Multiply(&tmp, &K1L, &fusion->P[0][1]);
    mat33Sub(&fusion->P[1][1], &tmp);

    mat33Multiply(&tmp, &K0L, &fusion->P[0][1]);
    mat33Sub(&fusion->P[0][1], &tmp);

    mat33Transpose(&fusion->P[1][0], &fusion->P[0][1]);

    struct Vec3 e = *z;
    vec3Sub(&e, &Bb);

    struct Vec3 dq;
    mat33Apply(&dq, &K[0], &e);


    struct Vec4 F[3];
    getF(F, &fusion->x0);

    // 4x3 * 3x1 => 4x1

    struct Vec4 q;
    q.x = fusion->x0.x + 0.5f * (F[0].x * dq.x + F[1].x * dq.y + F[2].x * dq.z);
    q.y = fusion->x0.y + 0.5f * (F[0].y * dq.x + F[1].y * dq.y + F[2].y * dq.z);
    q.z = fusion->x0.z + 0.5f * (F[0].z * dq.x + F[1].z * dq.y + F[2].z * dq.z);
    q.w = fusion->x0.w + 0.5f * (F[0].w * dq.x + F[1].w * dq.y + F[2].w * dq.z);

    fusion->x0 = q;
    quatNormalize(&fusion->x0);

    if (fusion->flags & FUSION_USE_MAG) {
        // accumulate gyro bias (causes self spin) only if not
        // game rotation vector
        struct Vec3 db;
        mat33Apply(&db, &K[1], &e);
        vec3Add(&fusion->x1, &db);
    }

    fusionCheckState(fusion);
}

#define ACC_TRUSTWORTHY(abs_norm_err)  ((abs_norm_err) < 1.f)
#define ACC_COS_CONV_FACTOR  0.01f
#define ACC_COS_CONV_LIMIT   3.f

int fusionHandleAcc(struct Fusion *fusion, int sensorType,
    const struct Vec3 *acc, float dT)
{
    if(!fusionInitComplete(fusion, sensorType, acc, dT)){
        return -1;
    }

    float norm2 = vec3NormSquared(acc);

    if (norm2 < FREE_FALL_THRESHOLD_SQ) {
        return -EINVAL;
    }

    float l = sqrtf(norm2);
    float l_inv = 1.0f / l;

    if (!(fusion->flags & FUSION_USE_GYRO)) {
        // geo mag mode
        // drive the Kalman filter with zero mean dummy gyro vector
        struct Vec3 w_dummy;

        // avoid (fabsf(norm_we) < kEps) in fusionPredict()
        initVec3(&w_dummy, fusion->x1.x + kEps, fusion->x1.y + kEps,
                 fusion->x1.z + kEps);

        updateDt(fusion, dT);
        fusionPredict(fusion, &w_dummy);
    }

    struct Mat33 R;
    fusionGetRotationMatrix(fusion, &R);

    if (!(fusion->flags & FUSION_USE_MAG) &&
        (fusion->fake_mag_decimation += dT) > FAKE_MAG_INTERVAL) {
        // game rotation mode, provide fake mag update to prevent
        // P to diverge over time
        struct Vec3 m;
        mat33Apply(&m, &R, &fusion->Bm);

        fusionUpdate(fusion, &m, &fusion->Bm,
                      fusion->param.mag_stdev);
        fusion->fake_mag_decimation = 0.f;
    }

    struct Vec3 unityA = *acc;
    vec3ScalarMul(&unityA, l_inv);

    float d = fabsf(l - NOMINAL_GRAVITY);
    float p;
    if (fusion->flags & FUSION_USE_GYRO) {
        float fc = 0;
        // Enable faster convergence
        if (ACC_TRUSTWORTHY(d)) {
            struct Vec3 aa;
            mat33Apply(&aa, &R, &fusion->Ba);
            float cos_err = vec3Dot(&aa, &unityA);
            cos_err = cos_err < (1.f - ACC_COS_CONV_FACTOR) ?
                (1.f - ACC_COS_CONV_FACTOR) : cos_err;
            fc = (1.f - cos_err) *
                    (1.0f / ACC_COS_CONV_FACTOR * ACC_COS_CONV_LIMIT);
        }
        p = fusion->param.acc_stdev * expf(3 * d - fc);
    } else {
        // Adaptive acc weighting (trust acc less as it deviates from nominal g
        // more), acc_stdev *= e(sqrt(| |acc| - g_nominal|))
        //
        // The weighting equation comes from heuristics.
        p = fusion->param.acc_stdev * expf(sqrtf(d));
    }

    fusionUpdate(fusion, &unityA, &fusion->Ba, p);

    return 0;

}

#define MAG_COS_CONV_FACTOR  0.02f
#define MAG_COS_CONV_LIMIT    2.f

int fusionHandleMag(struct Fusion *fusion, int sensorType,
    const struct Vec3 *mag)
{
    if(!fusionInitComplete(fusion, sensorType, mag, 0.0f)){
        return -1;
    }
    float magFieldSq = vec3NormSquared(mag);

    if (magFieldSq > MAX_VALID_MAGNETIC_FIELD_SQ
            || magFieldSq < MIN_VALID_MAGNETIC_FIELD_SQ) {
            printf("%s magFieldSq:%f\r\n",__func__,magFieldSq);
        return -EINVAL;
    }

    struct Mat33 R;
    fusionGetRotationMatrix(fusion, &R);

    struct Vec3 up;
    mat33Apply(&up, &R, &fusion->Ba);

    struct Vec3 east;
    vec3Cross(&east, mag, &up);

    if (vec3NormSquared(&east) < MIN_VALID_CROSS_PRODUCT_MAG_SQ) {
        return -EINVAL;
    }

    struct Vec3 north;
    vec3Cross(&north, &up, &east);

    float invNorm = 1.0f / vec3Norm(&north);
    vec3ScalarMul(&north, invNorm);

    float p = fusion->param.mag_stdev;

    if (fusion->flags & FUSION_USE_GYRO) {
        struct Vec3 mm;
        mat33Apply(&mm, &R, &fusion->Bm);
        float cos_err = vec3Dot(&mm, &north);
        cos_err = cos_err < (1.f - MAG_COS_CONV_FACTOR) ?
            (1.f - MAG_COS_CONV_FACTOR) : cos_err;

        float fc;
        fc = (1.f - cos_err) * (1.0f / MAG_COS_CONV_FACTOR * MAG_COS_CONV_LIMIT);
        p *= expf(-fc);
    }

    fusionUpdate(fusion, &north, &fusion->Bm, p);

    return 0;

}
int doSensorChang(struct Fusion *fusion, int sensorType,
    const struct Vec3 *vec, float dT)
{
    int ret = 0;
    switch(sensorType){
        case GYRO:
            ret = fusionHandleGyro(fusion, sensorType, vec, dT);
            break;
        case ACC:
            ret = fusionHandleAcc(fusion, sensorType, vec, dT);
            break;
        case MAG:
            ret = fusionHandleMag(fusion, sensorType, vec);
            break;
    }
    return ret;
}
void fusionGetRotationMatrix(const struct Fusion *fusion, struct Mat33 *R){
    quatToMatrix(R, &fusion->x0);
}
static float floatFromUint64(uint64_t v)
{
    uint32_t hi = v >> 32, lo = v;

    if (!hi) //this is very fast for cases where we fit into a uint32_t
        return(float)lo;
    else {
        return ((float)hi) * 4294967296.0f + (float)lo;
    }
}


int on_sensor_changed(uint8_t type, float *vec, long long dT)
{
    struct sensor_data data;

    data.type = type;

    dT = 1000000000ULL / 400;
    data.dT = floatFromUint64(dT) * 1e-9f;
    data.vec.x = vec[0];
    data.vec.y = vec[1];
    data.vec.z = vec[2];

    if(fusion_MsgQueueHandle == NULL) return -2;
    if(xQueueSend(fusion_MsgQueueHandle, &data, portMAX_DELAY) != pdPASS){
        return -1;
    }
    return 0;

}
int get_quaternion(float quaternion[])
{
    struct Fusion *fusion = &fusion_data;
    quaternion[0] = fusion->x0.x;
    quaternion[1] = fusion->x0.y;
    quaternion[2] = fusion->x0.z;
    quaternion[3] = fusion->x0.w;
    return 0;
}

static void fusion_thread_func(void const * argument)
{
    struct sensor_data data;
    struct Fusion *fusion = (struct Fusion *)argument;

    initFusion(fusion, FUSION_REINITIALIZE|FUSION_USE_GYRO|FUSION_USE_MAG );

    for(;;){
        if(fusion_MsgQueueHandle == NULL) continue;
        xQueueReceive( fusion_MsgQueueHandle, &data, portMAX_DELAY );
        doSensorChang(fusion, data.type, (struct Vec3 *)&data.vec, data.dT);
        //printf("%s [quat] x:%f, y:%f, z:%f, w:%f\r\n", __func__,
            //fusion->x0.x, fusion->x0.y,fusion->x0.z, fusion->x0.w);
    }
}

int fusion_init(void)
{
    osThreadDef(fusion_thread, fusion_thread_func,
                osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
    fusion_thread_handle = osThreadCreate(osThread(fusion_thread), &fusion_data);
    if(fusion_thread_handle == NULL){
        printf("create fusion_thread error\r\n");
    }
    osMessageQDef(fusion_MsgQueue, 4, sensor_data);
    fusion_MsgQueueHandle = osMessageCreate(osMessageQ(fusion_MsgQueue), NULL);
    return 0;
}


