#include <stdio.h>
#include "Fusion/HeadTracker.h"
#include "Fusion/Matrix.h"
#include "hlog_api.h"

typedef enum
{
    UNKNOWN = 0xFFFFFFFF,
    XA0n = 0x00000001,
    XB02 = 0x010102FF,
}  pcbid_t;

HeadTracker HeadTracker_Instance;
HeadTracker* HeadTracker_ = NULL;
//extern pcbid_t pcb_id;
bool tracking_on = false;
extern int sensorTimeOverflowCount;
extern int prevSensorTime;

void new_HeadTracker(HeadTracker* this_)
{
    this_->gyroBiasEstimator = NULL;
    this_->ekfToHeadTracker = this_->Instance_ekfToHeadTracker;
    this_->sensorToDisplay = this_->Instance_sensorToDisplay;
    this_->neckModelTranslation = this_->Instance_neckModelTranslation;
    this_->tmpHeadView = this_->Instance_tmpHeadView;
    this_->tmpHeadView2 = this_->Instance_tmpHeadView2;
    this_->initialSystemGyroBias = this_->Instance_initialSystemGyroBias;

    this_->tracker = &this_->Instance_OrientationEKF;
    new_OrientationEKF(this_->tracker);

    setGyroBiasEstimationEnabled(true, this_);
    setIdentityM(this_->neckModelTranslation, 0);

    this_->gyroBias = &this_->Instance_gyroBias;
    new_Vector3d(this_->gyroBias);

    this_->latestGyro = &this_->Instance_latestGyro;
    new_Vector3d(this_->latestGyro);

    this_->latestAcc = &this_->Instance_latestAcc;
    new_Vector3d(this_->latestAcc);

    this_->displayRotation = -3.4028235E38f;
    this_->neckModelFactor = 1.0f;
    this_->tracking = false;
    this_->latestGyroEventClockTimeNs = 0;
    this_->firstGyroValue = true;

    //get_pcbid(&pcb_id);
}

HeadTracker* get_HeadTracker_Instance()
{

    if (HeadTracker_ == NULL) {
        HeadTracker_ = &HeadTracker_Instance;
        new_HeadTracker(HeadTracker_);
    }

    return HeadTracker_;
}

void onSensorChanged(SensorEvent* event, HeadTracker* this_)  /* TODO */
{
    if (event->sensorType == 1) {

        Vector3d_set((float) event->values[0],
                     (float) event->values[1],
                     (float) event->values[2],
                     this_->latestAcc);

#ifdef DEBUG
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[onSensorChanged] processAcc[+] latestAcc x=%f, y=%f, z=%f\n",
                this_->latestAcc->x, this_->latestAcc->y, this_->latestAcc->z);
#endif /* DEBUG */

        processAcc(this_->latestAcc, event->timestamp, this_->tracker);

#ifdef DEBUG
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[onSensorChanged] processAcc[-] latestAcc x=%f, y=%f, z=%f\n",
                this_->latestAcc->x, this_->latestAcc->y, this_->latestAcc->z);
#endif /* DEBUG */

        if (this_->gyroBiasEstimator != NULL) {
            processAccelerometer(this_->latestAcc, event->timestamp,
                                 this_->gyroBiasEstimator);

#ifdef DEBUG
            hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[onSensorChanged] processAccelerometer latestAcc x=%f, y=%f, z=%f\n",
                    this_->latestAcc->x, this_->latestAcc->y, this_->latestAcc->z);
#endif /* DEBUG */
        }

    } else if (event->sensorType == 4) {
        /*this.latestGyroEventClockTimeNs = this.clock.nanoTime();TODO*/

        Vector3d_set((float) event->values[0],
                     (float) event->values[1],
                     (float) event->values[2],
                     this_->latestGyro);

        this_->firstGyroValue = false;

        if (this_->gyroBiasEstimator != NULL) {
            processGyroscope(this_->latestGyro, event->timestamp, this_->gyroBiasEstimator);
            getGyroBias(this_->gyroBias, this_->gyroBiasEstimator);
            Vector3d_sub(this_->latestGyro, this_->gyroBias, this_->latestGyro);
        }

#ifdef DEBUG
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[onSensorChanged] processGyro[+] latestGyro x=%f, y=%f, z=%f\n",
                this_->latestGyro->x, this_->latestGyro->y, this_->latestGyro->z);
#endif /* DEBUG */

        processGyro(this_->latestGyro, event->timestamp, this_->tracker);

#ifdef DEBUG
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[onSensorChanged] processGyro[-] latestGyro x=%f, y=%f, z=%f\n",
                this_->latestGyro->x, this_->latestGyro->y, this_->latestGyro->z);
#endif /* DEBUG */

    } else if (event->sensorType == 2) {

#ifdef DEBUG
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[onSensorChanged] processMag[+] Mag x=%f, y=%f, z=%f\n",
                event->values[0], event->values[1], event->values[2]);
#endif /* DEBUG */

        processMag(event->values, event->timestamp, this_->tracker);

#ifdef DEBUG
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[onSensorChanged] processMag[-] Mag x=%f, y=%f, z=%f\n",
                event->values[0], event->values[1], event->values[2]);
#endif /* DEBUG */
    }
}

void onAccuracyChanged(/*Sensor* sensor, */int accuracy,
        HeadTracker* this_) /* TODO */
{

}

void startTracking(HeadTracker* this_)
{
    if (!this_->tracking) {
        sensorTimeOverflowCount = 0;
        prevSensorTime = 0;
        OrientationEKF_reset(this_->tracker);

        if (this_->gyroBiasEstimator != NULL) {
            GyroscopeBiasEstimator_reset(this_->gyroBiasEstimator);
        }

        this_->firstGyroValue = true;
        tracking_on = true;

        this_->tracking = true;
    }
}

void resetTracker(HeadTracker* this_)
{
    OrientationEKF_reset(this_->tracker);
}

void stopTracking(HeadTracker* this_)
{
    if (this_->tracking) {
        tracking_on = false;
        this_->tracking = false;
    }
}

void setTrackingReset(unsigned char* item)
{
    HeadTracker* headTracker = get_HeadTracker_Instance();

    if (*item == 0) {
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "stopTracking !!!\n");
        stopTracking(headTracker);
    } else if (*item == 1) {
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "startTracking !!!\n");
        startTracking(headTracker);
    }
}

void setNeckModelEnabled(bool enabled, HeadTracker* this_)
{
    if (enabled) {
        setNeckModelFactor(1.0F, this_);
    } else {
        setNeckModelFactor(0.0F, this_);
    }
}

float getNeckModelFactor(HeadTracker* this_)
{
    /*Object var1 = this.neckModelFactorMutex;
    synchronized(this.neckModelFactorMutex) {
    TODO*/
    return this_->neckModelFactor;
    /*TODO
        }
    */
}

void setNeckModelFactor(float factor, HeadTracker* this_)
{
    /*Object var2 = this.neckModelFactorMutex; TODO*/
    /*synchronized(this.neckModelFactorMutex) { TODO*/
    if (factor >= 0.0F && factor <= 1.0F) {
        this_->neckModelFactor = factor;
    } else {
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "factor should be within [0.0, 1.0]\n");
    }

    /*TODO
        }
    */
}

void setGyroBiasEstimationEnabled(bool enabled, HeadTracker* this_)
{
    /*Object var2 = this.gyroBiasEstimatorMutex;TODO*/
    /*synchronized(this.gyroBiasEstimatorMutex) {TODO*/
    if (!enabled) {
        this_->gyroBiasEstimator = NULL;
    } else if (this_->gyroBiasEstimator == NULL) {
        this_->gyroBiasEstimator = &this_->Instance_gyroBiasEstimator;
        new_GyroscopeBiasEstimator(this_->gyroBiasEstimator);
    }

    /*
        }
    */
}

bool getGyroBiasEstimationEnabled(HeadTracker* this_)
{
    /*
    Object var1 = this.gyroBiasEstimatorMutex;
    synchronized(this.gyroBiasEstimatorMutex) {
    TODO*/
    return this_->gyroBiasEstimator != NULL;
    /*TODO
    }
    */
}

void plugin_getLastHeadView(float headView[], int length, int offset,
                            HeadTracker* this_)
{
    if (offset + 16 > length) {
        hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_FUSION, "Not enough space to write the result\n");
    } else {
        float rotation = 0.0F;

        switch (1/*this.display.getRotation() TODO*/) {
        case 0:
            rotation = 0.0F;
            break;

        case 1:
            rotation = 90.0F;
            break;

        case 2:
            rotation = 180.0F;
            break;

        case 3:
            rotation = 270.0F;
        }

        if (rotation != this_->displayRotation) {
            this_->displayRotation = rotation;
            setRotateEulerM(this_->sensorToDisplay, 0, 0.0F, 0.0F, -90.0F);
            setRotateEulerM(this_->ekfToHeadTracker, 0, -90.0F, 0.0F, rotation);
        }

        if (!isReady(this_->tracker)) {
            return;
        }

        float* mat = plugin_getPredictedGLMatrix(0.0f, this_->tracker);
        // float* mat = getPredictedGLMatrix(0.057999998331069946, this_->tracker);

        for (int i = 0; i < length; ++i) {
            this_->tmpHeadView[i] = (float) mat[i];
        }

        multiplyMM(this_->tmpHeadView2, 0, this_->sensorToDisplay, 0,
                   this_->tmpHeadView, 0);

        multiplyMM(headView, offset, this_->tmpHeadView2, 0, this_->ekfToHeadTracker,
                   0);
    }
}

void getLastHeadView(float headView[], int length, int offset,
                     HeadTracker* this_)
{

    if (offset + 16 > length) {
        hlog_printf(HLOG_LVL_WARNING, HLOG_TAG_FUSION, "Not enough space to write the result\n");
    } else {
        float rotation = 0.0F;

        switch (1/*this.display.getRotation() TODO*/) {
        case 0:
            rotation = 0.0F;
            break;

        case 1:
            rotation = 90.0F;
            break;

        case 2:
            rotation = 180.0F;
            break;

        case 3:
            rotation = 270.0F;
        }

        if (rotation != this_->displayRotation) {
            this_->displayRotation = rotation;
            setRotateEulerM(this_->sensorToDisplay, 0, 0.0F, 0.0F, -rotation);
            setRotateEulerM(this_->ekfToHeadTracker, 0, -90.0F, 0.0F, rotation);
        }

        /*synchronized(this.tracker) {TODO*/
        if (!isReady(this_->tracker)) {
            return;
        }

        float secondsSinceLastGyroEvent =
            0.0; /*(float) TimeUnit.NANOSECONDS.toSeconds(
                                               this.clock.nanoTime() - this.latestGyroEventClockTimeNs); TODO*/

        float secondsToPredictForward = secondsSinceLastGyroEvent +
                                        0.057999998331069946;

        float* mat = getPredictedGLMatrix(secondsToPredictForward, this_->tracker);

        for (int i = 0; i < length; ++i) {
#ifdef NORMAL
            headView[i] = (float) mat[i];
#ifdef DEBUG
            hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "headView[%d]=%f\n", i , headView[i]);
#endif /* DEBUG */
#else
            this_->tmpHeadView[i] = (float) mat[i];
#endif /* NORMAL */

#ifndef NORMAL
#ifdef DEBUG
            hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "getLastHeadView getPredictedGLMatrix tmpHeadView[%d]=%f\n", i ,
                   this_->tmpHeadView[i]);
#endif /* DEBUG */
#endif /* NORMAL */
        }

        /*}*/

#ifndef NORMAL
        multiplyMM(this_->tmpHeadView2, 0, this_->sensorToDisplay, 0,
                   this_->tmpHeadView, 0);
        multiplyMM(headView, offset, this_->tmpHeadView2, 0, this_->ekfToHeadTracker,
                   0);
        setIdentityM(this_->neckModelTranslation, 0);

        translateM(this_->neckModelTranslation, 0, 0.0F,
                   -this_->neckModelFactor * 0.075F, this_->neckModelFactor * 0.08F);

        multiplyMM(this_->tmpHeadView, 0, this_->neckModelTranslation, 0, headView,
                   offset);

        translateM_l(headView, offset, this_->tmpHeadView, 0, 0.0F,
                     this_->neckModelFactor * 0.075F, 0.0F);
#endif /* NORMAL */
    }
}

void setGyroBiasEstimator(GyroscopeBiasEstimator* estimator,
                          HeadTracker* this_) /* TODO */
{
    /*Object var2 = this.gyroBiasEstimatorMutex;
    synchronized(this.gyroBiasEstimatorMutex) {
    TODO*/
    this_->gyroBiasEstimator = estimator;
    /*}*/
}

void getSensorFromWorld(double* mat, HeadTracker* this_)
{
    for (int i = 0; i < 9; ++i) {
        mat[i] = this_->tracker->so3SensorFromWorld->m[i];
    }
}

void getLastGyro(double* last_gyro, HeadTracker* this_)
{
    last_gyro[0] = this_->tracker->lastGyro->x;
    last_gyro[1] = this_->tracker->lastGyro->y;
    last_gyro[2] = this_->tracker->lastGyro->z;
}

void getLastGyro_angleAcc(double* last_angleAcc, HeadTracker* this_)
{
    last_angleAcc[0] = this_->tracker->lastAngleAcc->x;
    last_angleAcc[1] = this_->tracker->lastAngleAcc->y;
    last_angleAcc[2] = this_->tracker->lastAngleAcc->z;
}

