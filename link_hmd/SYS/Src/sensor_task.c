/**
 ******************************************************************************
 * File Name          : sensor_task.c
 * Description        : Sensor task to get sensor data
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "sensor_task.h"
#include "Fusion/base.h"
#ifdef CARDBOARD_FUSION
#include "Fusion/SensorEvent.h"
#include "Fusion/HeadTracker.h"
#include "Fusion/HeadTransform.h"
#include "Fusion/imu_fusion.h"
#include "Fusion/Matrix.h"
#include "Fusion/Vector3d.h"
#endif /* CARDBOARD_FUSION */
#include "htc_3dof_transfer_service.h"
#include "misc_data.h"

#ifdef CARDBOARD_FUSION
#define SENSOR_TIME_OVERFLOW_MAX powerInt(2, 24)
#endif /* CARDBOARD_FUSION */
#include "fusion.h"

osThreadId sensorTaskHandle;
BaseType_t xSensorTaskWoken = pdFALSE;
void* SENSOR_X_0_handle = NULL;
void* SENSOR_G_0_handle = NULL;
void* MAGNETIC_handle = NULL;
extern pcbid_t pcb_id;

#ifdef CARDBOARD_FUSION
SensorEvent accelEvent;
SensorEvent gyroEvent;
SensorEvent MagEvent;
HeadTransform Instance_headTransform;
extern int imu_log;
extern bool tracking_on;
int sensorTimeOverflowCount = 0;
us32 prevSensorTime = 0;
#endif /* CARDBOARD_FUSION */

void StartSensorTask(void const* argument);
static void initializeAllSensors(void);

#ifdef CARDBOARD_FUSION
#define min(a,b)  (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

struct rawdata_buf {
    double x[5], y[5], z[5];
    double sum[3];
    size_t index;
};

typedef struct quat_t_ {
    double w;
    double x;
    double y;
    double z;
} quat_t ;

/* enable it if you need dump sample rate log at each second */
//#define SENSOR_TASK_DUMP_SAMPLE_RATE_LOG

#ifdef SENSOR_TASK_DUMP_SAMPLE_RATE_LOG
#define SENSOR_TASK_DUMP_DURING     configTICK_RATE_HZ
static osTimerDef_t dump_sample_rate_timer_def = { 0 };
static osTimerId dump_sample_rate_timer_id = { 0 };
static int count_imu = 0;
static long long TimeStamp_E = 0;

static void sensor_task_dump_sample_rate_log(void const* argument)
{
#ifdef PLUGIN
    hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "IMU Sample rate = %d (times/sec), Spent Fusion time %f ms\r\n",
           count_imu, (float)TimeStamp_E / count_imu);

    count_imu = 0;
    TimeStamp_E = 0;
#endif /* PLUGIN */
}

static void sensor_task_dump_sample_rate_init()
{
    dump_sample_rate_timer_def.ptimer = sensor_task_dump_sample_rate_log;
    dump_sample_rate_timer_id = osTimerCreate(
            &dump_sample_rate_timer_def,
            osTimerPeriodic,
            NULL);
    osTimerStart(dump_sample_rate_timer_id, SENSOR_TASK_DUMP_DURING);
}
#endif /* SENSOR_TASK_DUMP_REPORT_RATE_LOG */

static int powerInt(int base, int n)
{
    int i;
    int pow = 1;

    for (i = 1; i <= n; i++) {
        pow = pow * base;
    }

    return pow;
}

static double power(double base, int n)
{
    int i;
    double pow = 1.0;

    for (i = 1; i <= n; i++) {
        pow = pow * base;
    }

    return pow;
}

static short FloatToQFormatInteger(const double x, const int m, const int n)
{
    return (((x) < (0.0)) ? ((int)(power(2, n) * (x) - 0.5)) : (min(32767,
            (int)((32767.0 / (power(2, m))) * (x) + 0.5))));
}
#endif /* CARDBOARD_FUSION */

extern void bmi160_postinit(void);
#if SENSOR_LATENCY_TEST
#define  DWT_CR                 *(volatile uint32_t *)0xE0001000
#define  DWT_CYCCNT             *(volatile uint32_t *)0xE0001004
#define  DEM_CR                 *(volatile uint32_t *)0xE000EDFC
#define  DBGMCU_CR              *(volatile uint32_t *)0xE0042004
#define  DEM_CR_TRCENA          (1 << 24)
#define  DWT_CR_CYCCNTENA       (1 <<  0)

uint32_t latency_data[SENSOR_LATENCY_TEST_COUNT][SENSOR_LATENCY_TEST_POINT] = {0};
volatile uint32_t latency_index = 0;
uint32_t latency_lock = 0;


void sensor_latency_test_init(void)
{
    DEM_CR         |=  DEM_CR_TRCENA;
    DWT_CR         &= ~DWT_CR_CYCCNTENA;
    DWT_CYCCNT      = 0u;
    DWT_CR         |= DWT_CR_CYCCNTENA;
    latency_index = 0;
}

#endif
void sensorTaskInit(void)
{
    osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 1024);
    sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

    initializeAllSensors();
    sensorEnable(ACC);
    sensorEnable(GYRO);
    sensorSetOdr(ACC, ODR_MID_HIGH); // 2g 400hz
    sensorSetOdr(GYRO, ODR_MID_HIGH); // 2000dps 400hz

    // enable OSR4 mode
    us8 accel_bw = 0;
    if (bmi160_set_accel_bw(accel_bw) != BMI160_SUCCESS) {
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "bmi160_set_accel_bw (%d) fail\n", accel_bw);
    }

    // enable OSR4 mode
    us8 gyro_bw = 0;
    if (bmi160_set_gyro_bw(gyro_bw) != BMI160_SUCCESS) {
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "bmi160_set_gyro_bw (%d) fail\n", gyro_bw);
    }

    if (BSP_MAGNETO_Init(MAGNETO_SENSORS_AUTO, &MAGNETIC_handle) != COMPONENT_OK) {
        mag_err("magnetic sensor  initial fail\n");
    }
    sensorSetOdr(MAG, ODR_HIGH);
    sensorEnable(MAG);
    bmi160_postinit();
#ifdef SENSOR_TASK_DUMP_SAMPLE_RATE_LOG
    sensor_task_dump_sample_rate_init();
#endif /* SENSOR_TASK_DUMP_SAMPLE_RATE_LOG */
#if SENSOR_LATENCY_TEST
    sensor_latency_test_init();
#endif
    fusion_init();

}

void sensorIsr(uint16_t GPIO_Pin)
{
    if (sensorTaskHandle) {

        #if SENSOR_LATENCY_TEST
        latency_data[latency_index][0] = DWT_CYCCNT;
        #endif
        xTaskNotifyFromISR(sensorTaskHandle, GPIO_Pin, eSetBits, &xSensorTaskWoken);
        portYIELD_FROM_ISR(xSensorTaskWoken);
    }
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void)
{
    if (BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &SENSOR_X_0_handle) != COMPONENT_OK) {
        gsensor_err("acc init fail\n");
    }

    if (BSP_GYRO_Init(GYRO_SENSORS_AUTO, &SENSOR_G_0_handle) != COMPONENT_OK) {
        gsensor_err("gyro init fail\n");
    }
}

void sensorEnable(SENSOR_ID_t sensor_id)
{
    switch (sensor_id) {
    case ACC :
        if (BSP_ACCELERO_Sensor_Enable(SENSOR_X_0_handle) != COMPONENT_OK) {
            gsensor_err("G-Sensor enable fail\n");
        }
        break;

    case GYRO :
        if (BSP_GYRO_Sensor_Enable(SENSOR_G_0_handle) != COMPONENT_OK) {
            gsensor_err("Gyro Sensor enable fail\n");
        }
        break;

    case MAG :
        if (BSP_MAGNETO_Sensor_Enable(MAGNETIC_handle) != COMPONENT_OK) {
            mag_err("magnetic sensor enable fail\n");
        }
        break;

    default :
        break;
    }
}

int sensorSuspendResume(SensorMode_t Sensor_mode)
{
    if ((MAGNETIC_handle == NULL)&&(SENSOR_X_0_handle == NULL)) {
        hlog_printf(HLOG_LVL_ERR, HLOG_TAG_SENSOR, "please call after sensor initial done\n");
        return -2;
    }
    if (Sensor_mode == SUSPEND) {
        /*magnetic sensor should suspend before accleration and gyro sensor when suspend flow*/
        if (BSP_MAGNETO_Set_Mode(MAGNETIC_handle, SUSPEND) != COMPONENT_OK) {
            mag_err("magnetic sensor suspend fail\n");
            return -1;
        }
        if (BSP_ACCELERO_Set_Mode(SENSOR_X_0_handle, SUSPEND) != COMPONENT_OK) {
            gsensor_err("accleration and gyro sensor suspend fail\n");
            return -1;
        }

    } else if (Sensor_mode == RESUME) {
        /*magnetic sensor should resume after accleration and gyro sensor when resume flow*/
        if (BSP_ACCELERO_Set_Mode(SENSOR_X_0_handle, RESUME) != COMPONENT_OK) {
            gsensor_err("accleration and gyro sensor resume fail\n");
            return -1;
        }
        if (BSP_MAGNETO_Set_Mode(MAGNETIC_handle, RESUME) != COMPONENT_OK) {
            mag_err("magnetic sensor resume fail\n");
            return -1;
        }
    }

    return 0;
}

void sensorDisable(SENSOR_ID_t sensor_id)
{
    switch (sensor_id) {
    case ACC :
        if (BSP_ACCELERO_Sensor_Disable(SENSOR_X_0_handle) != COMPONENT_OK) {
            gsensor_err("G-Sensor disable fail\n");
        }
        break;

    case GYRO :
        if (BSP_GYRO_Sensor_Disable(SENSOR_G_0_handle) != COMPONENT_OK) {
            gsensor_err("Gyro Sensor disable fail\n");
        }
        break;

    case MAG :
        if (BSP_MAGNETO_Sensor_Disable(MAGNETIC_handle) != COMPONENT_OK) {
            mag_err("magnetic sensor disable fail\n");
        }
        break;

    default :
        break;
    }
}

/*set sensor ODR*/
void sensorSetOdr(SENSOR_ID_t sensor_id, SensorOdr_t odr)
{
    switch (sensor_id) {
    case ACC :
        BSP_ACCELERO_Set_ODR(SENSOR_X_0_handle, odr);
        break;

    case GYRO :
        BSP_GYRO_Set_ODR(SENSOR_G_0_handle, odr);
        break;

    case MAG :
        BSP_MAGNETO_Set_ODR(MAGNETIC_handle, odr);
        break;

    default :
        break;
    }
}

/*get sensor ODR*/
void sensorGetOdr(SENSOR_ID_t sensor_id, float* odr)
{
    switch (sensor_id) {
    case ACC :
        BSP_ACCELERO_Get_ODR(SENSOR_X_0_handle, odr);
        break;

    case GYRO :
        BSP_GYRO_Get_ODR(SENSOR_G_0_handle, odr);
        break;

    case MAG :
        BSP_MAGNETO_Get_ODR(MAGNETIC_handle, odr);
        break;

    default :
        break;
    }
}

void sensorGetData(SENSOR_ID_t sensor_id, SensorAxes_t* sensor_data)
{
    switch (sensor_id) {
    case ACC :
        BSP_ACCELERO_Get_Axes(SENSOR_X_0_handle, sensor_data);
        break;

    case MAG :
        BSP_MAGNETO_Get_Axes(MAGNETIC_handle, sensor_data);
        break;

    default :
        break;
    }
}

void sensorGetGyroData(SENSOR_ID_t sensor_id, SensorAxes_gyro_t* sensor_data)
{
    SensorAxes_t  sensor_datatmp;

    switch (sensor_id) {
    case GYRO :
        BSP_GYRO_Get_Axes(SENSOR_G_0_handle, &sensor_datatmp);
        sensor_data->AXIS_X = (float)sensor_datatmp.AXIS_X / 3600;
        sensor_data->AXIS_Y = (float)sensor_datatmp.AXIS_Y / 3600;
        sensor_data->AXIS_Z = (float)sensor_datatmp.AXIS_Z / 3600;
        break;

    default :
        break;
    }
}

void magneticSetUserCalibration(uint8_t enable)
{
    BSP_MAGNETO_Set_User_Calibration(MAGNETIC_handle,enable);
}

void magneticGetUserCalibration(uint8_t *info,uint8_t length)
{
    BSP_MAGNETO_Get_User_Calibration(MAGNETIC_handle,info,length);
}

char gsensorDebugFlag = 0;
#pragma optimize=none
void StartSensorTask(void const* argument)
{
#ifdef DEBUG
    float prev_quaternion[4] = {0.0};
#endif

#ifdef CARDBOARD_FUSION
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    us32 nowSensorTime = 0;
    long long finalTimeStamp = 0;
#ifdef SENSOR_TASK_DUMP_SAMPLE_RATE_LOG
    long long TimeStamp_S;
#endif /* SENSOR_TASK_DUMP_SAMPLE_RATE_LOG */
    HeadTracker* headTracker = get_HeadTracker_Instance();
    HeadTransform* headTransform = &Instance_headTransform;
    new_HeadTransform(headTransform);
#ifdef DEBUG
    float eulerAngles[3] = {0.0, 0.0, 0.0};
#endif /* DEBUG */
    imu_t imu_data;
    long serial = 0;
#endif /* CARDBOARD_FUSION */

    float sensorToDisplay[9];
    Vector3d lastGyro_v;
    Vector3d lastGyro_angleAcc_v;
    SensorAxes_t acceleration;
    SensorAxes_gyro_t angular_velocity;
    SensorAxes_t magnetometer;

    uint32_t ulNotificationValue;
    uint8_t DrdyFlag;

    get_pcbid(&pcb_id);
    setRotateEuler3x3M(sensorToDisplay, 0, 0.0F, 0.0F, -90.0F);
    for (;;) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);
        xSensorTaskWoken = pdFALSE;
        #if SENSOR_LATENCY_TEST
        latency_data[latency_index][1] = DWT_CYCCNT;
        #endif

        if ((ulNotificationValue & GPIO_PIN_1) == GPIO_PIN_1) {
            bmi160_read_reg(BMI160_USER_STAT_ADDR,&DrdyFlag,1);
            if ((DrdyFlag & 0xC0) == 0xC0) {
                //hlog_printf(HLOG_LVL_INFO, HLOG_TAG_SENSOR, "ACC+GYRO\n");
                sensorGetData(ACC, &acceleration);//mg
                sensorGetGyroData(GYRO, &angular_velocity);//mdps
            }
            if ((DrdyFlag & 0x20) == 0x20) {
                //hlog_printf(HLOG_LVL_INFO, HLOG_TAG_SENSOR, "MAG BMI\n");
                sensorGetData(MAG, &magnetometer);
            }
            #if SENSOR_LATENCY_TEST
            latency_data[latency_index][2] = DWT_CYCCNT;
            #endif

            if (gsensorDebugFlag == 1) {
                gsensor_emerg("SENSOR_X_0_handle = 0x%x SENSOR_G_0_handle = 0x%x\n", SENSOR_X_0_handle,SENSOR_G_0_handle);
                gsensor_emerg("Acc x = %dmg y = %dmg z = %dmg\n", acceleration.AXIS_X, acceleration.AXIS_Y, acceleration.AXIS_Z);
                gsensor_emerg("GYRO x = %fmdpsy = %fmdps z = %fmdps\n", angular_velocity.AXIS_X, angular_velocity.AXIS_Y, angular_velocity.AXIS_Z);
            }

            bmi160_get_sensor_time(&nowSensorTime);
            if (nowSensorTime < prevSensorTime) {
                sensorTimeOverflowCount++;
                hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "sensorTimeOverflowCount: %d\n", sensorTimeOverflowCount);
            }
            finalTimeStamp = (long long) (sensorTimeOverflowCount * SENSOR_TIME_OVERFLOW_MAX + nowSensorTime) * 39 * 1000; // naro second
            prevSensorTime = nowSensorTime;
        }

        if ((ulNotificationValue & GPIO_PIN_2) == GPIO_PIN_2) {
            //hlog_printf(HLOG_LVL_INFO, HLOG_TAG_SENSOR, "MAG INT\n");
            sensorGetData(MAG, &magnetometer);

        }
        #if 0
            mag_info("MAG x = %d y = %d z = %d\n", magnetometer.AXIS_X, magnetometer.AXIS_Y, magnetometer.AXIS_Z);
        #endif /* DEBUG */

#ifdef CARDBOARD_FUSION

#ifdef SENSOR_TASK_DUMP_SAMPLE_RATE_LOG
        TimeStamp_S = HAL_GetTick();
#endif /* SENSOR_TASK_DUMP_SAMPLE_RATE_LOG */

        if (XA0n == pcb_id) {
            ax = ((float)acceleration.AXIS_X * 9.8 / 1000.0);
            ay = -((float)acceleration.AXIS_Y * 9.8 / 1000.0);
            az = -((float)acceleration.AXIS_Z * 9.8 / 1000.0);

            // Degree transfer to Rad
            gx = (((float)angular_velocity.AXIS_X)) * 3.14 / 180.0;
            gy = -(((float)angular_velocity.AXIS_Y)) * 3.14 / 180.0;
            gz = -(((float)angular_velocity.AXIS_Z)) * 3.14 / 180.0;
        } else {
            ax = ((float)acceleration.AXIS_Y * 9.8 / 1000.0);
            ay = ((float)acceleration.AXIS_X * 9.8 / 1000.0);
            az = -((float)acceleration.AXIS_Z * 9.8 / 1000.0);

            // Degree transfer to Rad
            gx = (((float)angular_velocity.AXIS_Y)) * 3.14 / 180.0;
            gy = (((float)angular_velocity.AXIS_X)) * 3.14 / 180.0;
            gz = -(((float)angular_velocity.AXIS_Z)) * 3.14 / 180.0;
        }

#if 0
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "TimeStamp: %lld, ACC: %f,%f,%f, Gyro: %f,%f,%f\r\n", finalTimeStamp, ax, ay, az, gx, gy, gz);
#endif /* DEBUG */
        tracking_on = true;

        if (true == tracking_on) {
            accelEvent.sensorType = 1;
            accelEvent.values[0] = ax;
            accelEvent.values[1] = ay;
            accelEvent.values[2] = az;
            accelEvent.timestamp = finalTimeStamp;
            onSensorChanged(&accelEvent, headTracker);
            on_sensor_changed(accelEvent.sensorType, accelEvent.values,
                            accelEvent.timestamp);

            gyroEvent.sensorType = 4;
            gyroEvent.values[0] = gx;
            gyroEvent.values[1] = gy;
            gyroEvent.values[2] = gz;
            gyroEvent.timestamp = finalTimeStamp;
            onSensorChanged(&gyroEvent, headTracker);
            on_sensor_changed(gyroEvent.sensorType, gyroEvent.values,
                            gyroEvent.timestamp);
            mx = (float)(magnetometer.AXIS_X/1000.0);
            my = (float)(magnetometer.AXIS_Y/1000.0);
            mz = (float)(magnetometer.AXIS_Z/1000.0);
            #if 1
                mag_info("MAG x = %f y = %f z = %f\n", mx, my, mz);
            #endif /* DEBUG */

            MagEvent.sensorType = 2;
            MagEvent.values[0] = mx;
            MagEvent.values[1] = my;
            MagEvent.values[2] = mz;
            MagEvent.timestamp = finalTimeStamp;
            on_sensor_changed(MagEvent.sensorType, MagEvent.values,
                            MagEvent.timestamp);
        }

#ifdef PLUGIN
        double last_gyro[3] = {0.0};
        double last_gyro_angleacc[3] = {0.0};
        float quaternion[4] = {0.0};

        getLastGyro(last_gyro, headTracker);
        getLastGyro_angleAcc(last_gyro_angleacc, headTracker);
        plugin_getLastHeadView(getHeadView_l(headTransform), 16, 0, headTracker);

#ifdef DEBUG
        getEulerAngles(eulerAngles, 3, 0 , headTransform);

        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "Picth: %f; Yaw: %f;Roll: %f \n",
                ((eulerAngles[0] * 180.0) / 3.14),
                ((eulerAngles[1] * 180.0) / 3.14),
                ((eulerAngles[2] * 180.0) / 3.14));
#endif /* DEBUG */

        getQuaternion(quaternion, 4, 0, headTransform);
    //printf("%s [quat] x:%f, y:%f, z:%f, w:%f\r\n", __func__,quaternion[0],
            //quaternion[1],quaternion[2], quaternion[3]);

#ifdef DEBUG
        float m = sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);

        if (fabs(1-m) > 0.005) {
            hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "Abnormal Quaternion(w:x:y:z) = (%.5f:%.5f:%.5f:%.5f) acc : %f,%f,%f, gyro : %f,%f,%f \n",
                    quaternion[3], quaternion[0], quaternion[1], quaternion[2], ax, ay, az, gx, gy, gz);
        }
#endif /* DEBUG */

#ifdef DEBUG
        float diff_w = quaternion[3] - prev_quaternion[3];
        float diff_x = quaternion[0] - prev_quaternion[0];
        float diff_y = quaternion[1] - prev_quaternion[1];
        float diff_z = quaternion[2] - prev_quaternion[2];

        float diff = sqrt(diff_w * diff_w + diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[TimeStamp: %lld] Qdiff = %.5f : Quaternion(w:x:y:z) = (%.5f:%.5f:%.5f:%.5f) acc : %f,%f,%f, gyro : %f,%f,%f \n",
                finalTimeStamp, diff, quaternion[3], quaternion[0], quaternion[1], quaternion[2], ax, ay, az, gx, gy, gz);

        prev_quaternion[3] = quaternion[3];
        prev_quaternion[0] = quaternion[0];
        prev_quaternion[1] = quaternion[1];
        prev_quaternion[2] = quaternion[2];
#endif /* DEBUG */

#ifdef SENSOR_TASK_DUMP_SAMPLE_RATE_LOG
        TimeStamp_E = (HAL_GetTick() - TimeStamp_S) + TimeStamp_E;
        count_imu++;
#endif /* SENSOR_TASK_DUMP_SAMPLE_RATE_LOG */
        get_quaternion(quaternion);

        // quaternion to Q14, -2?x?1.9999390
        imu_data.q_w = FloatToQFormatInteger(quaternion[3] , 1, 14);
        imu_data.q_x = FloatToQFormatInteger(quaternion[0] , 1, 14);
        imu_data.q_y = FloatToQFormatInteger(quaternion[1] , 1, 14);
        imu_data.q_z = FloatToQFormatInteger(quaternion[2] , 1, 14);

        // accel to Q9, -64?x?63.9980469
        imu_data.a_x = FloatToQFormatInteger(ax/*avg_ax*/ , 6, 9);
        imu_data.a_y = FloatToQFormatInteger(ay/*avg_ay*/ , 6, 9);
        imu_data.a_z = FloatToQFormatInteger(az/*avg_az*/ , 6, 9);

        // Convert from sensor coordinate frame to display orientation.
        lastGyro_v.x = sensorToDisplay[0] * last_gyro[0] + sensorToDisplay[3] * last_gyro[1] + sensorToDisplay[6] * last_gyro[2];
        lastGyro_v.y = sensorToDisplay[1] * last_gyro[0] + sensorToDisplay[4] * last_gyro[1] + sensorToDisplay[7] * last_gyro[2];
        lastGyro_v.z = sensorToDisplay[2] * last_gyro[0] + sensorToDisplay[5] * last_gyro[1] + sensorToDisplay[8] * last_gyro[2];
        // gyro to Q9, -64?x?63.9980469
        imu_data.g_x = FloatToQFormatInteger(lastGyro_v.x , 6, 9);
        imu_data.g_y = FloatToQFormatInteger(lastGyro_v.y , 6, 9);
        imu_data.g_z = FloatToQFormatInteger(lastGyro_v.z , 6, 9);

        // Convert from sendor coordinate frame to display orientation.
        lastGyro_angleAcc_v.x = sensorToDisplay[0] * last_gyro_angleacc[0] + sensorToDisplay[3] * last_gyro_angleacc[1] + sensorToDisplay[6] * last_gyro_angleacc[2];
        lastGyro_angleAcc_v.y = sensorToDisplay[1] * last_gyro_angleacc[0] + sensorToDisplay[4] * last_gyro_angleacc[1] + sensorToDisplay[7] * last_gyro_angleacc[2];
        lastGyro_angleAcc_v.z = sensorToDisplay[2] * last_gyro_angleacc[0] + sensorToDisplay[5] * last_gyro_angleacc[1] + sensorToDisplay[8] * last_gyro_angleacc[2];
        // gyro angle acc to Q9, -64?x?63.9980469
        imu_data.g_angleacc_x = FloatToQFormatInteger(lastGyro_angleAcc_v.x , 6, 9);
        imu_data.g_angleacc_y = FloatToQFormatInteger(lastGyro_angleAcc_v.y , 6, 9);
        imu_data.g_angleacc_z = FloatToQFormatInteger(lastGyro_angleAcc_v.z , 6, 9);

        imu_data.serial = serial++;
        imu_data.timeStamp = (long long)(finalTimeStamp / 1000);

#ifdef DEBUG
        hlog_printf(HLOG_LVL_INFO, HLOG_TAG_FUSION, "[Quaternion]w: %d, x: %d, y: %d, z: %d\n", imu_data.q_w, imu_data.q_x, imu_data.q_y, imu_data.q_z);
#endif /* DEBUG */

        htc_3dof_tran_service_send_data((uint8_t*)&imu_data, sizeof(imu_t));
        #if SENSOR_LATENCY_TEST
        latency_data[latency_index][3] = DWT_CYCCNT;
        if(!latency_lock){
            if(++latency_index >= SENSOR_LATENCY_TEST_COUNT){
                latency_index = 0;
            }
        }
        #endif

#endif /* PLUGIN */
#endif /* CARDBOARD_FUSION */
    }
}
