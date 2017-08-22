#include "imu.h"
#include "math.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define Ki      0.005f
#define Kp      2.0f
#define halfT   0.5f

float exInt = 0, eyInt = 0, ezInt = 0;

static imudata Data = {0};
osThreadId imu_thread_handle;
osMessageQId imuMsgQueueHandle;
#define min(a,b)  (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
float invSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x; // get bits for floating value
    i = 0x5f375a86 - (i>>1); // gives initial guess
    x = *(float*)&i; // convert bits back to float
    x = x * (1.5f - xhalf*x*x); // Newton step
    return x;
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

int imu_get_euler_angles(float * q)
{
    float pitch, roll, yaw;
    roll = (float) atan2(2*(q[2]*q[3]+q[0]*q[1]),
                         (q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]));
    pitch = (float) asin((2*(q[0]*q[2]-q[1]*q[3]))/(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));
    yaw = (float) atan2(2*(q[1]*q[2]+q[0]*q[3]),
                        (q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]));
    printf("pitch:%f, yaw:%f, roll:%f\r\n", pitch, yaw, roll);

    return 0;
}
void IMUupdate() {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float ax,ay,az,gx,gy,gz;
    float q0,q1,q2,q3;
    float q0temp,q1temp,q2temp,q3temp;

    ax=Data.pSensor->accel[0];
    ay=Data.pSensor->accel[1];
    az=Data.pSensor->accel[2];

    gx=Data.pSensor->gyro[0];
    gy=Data.pSensor->gyro[1];
    gz=Data.pSensor->gyro[2];

    q0=Data.q[0];
    q1=Data.q[1];
    q2=Data.q[2];
    q3=Data.q[3];

    // normalise the measurements
    /* 将ACC三维向量转成单位向量 */
    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    //*Note1
    // estimated direction of gravity
    /* 将机体坐标系转换成重力单位向量*/
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    //*Note2
    /* error is sum of cross product between reference direction of field
        and direction measured by sensor   */
    /* ACC 单位向量 与 估算姿态转换成的重力向量 做叉乘 --> 误差向量*/
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    // integral error scaled integral gain
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;

    // adjusted gyroscope measurements， PID
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    q0temp=q0;
    q1temp=q1;
    q2temp=q2;
    q3temp=q3;

    //*Note3
    // integrate quaternion rate and normalise
    q0 = q0temp + (-q1temp *gx - q2temp *gy - q3temp *gz)*halfT;
    q1 = q1temp + (q0temp *gx + q2temp *gz - q3temp *gy)*halfT;
    q2 = q2temp + (q0temp *gy - q1temp *gz + q3temp *gx)*halfT;
    q3 = q3temp + (q0temp *gz + q1temp *gy - q2temp *gx)*halfT;

    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    Data.q[0]=q0;
    Data.q[1]=q1;
    Data.q[2]=q2;
    Data.q[3]=q3;
    printf("[Quaternion]w: %d, x: %d, y: %d, z: %d\n",
            FloatToQFormatInteger(q3,1,14),
            FloatToQFormatInteger(q0,1,14),
            FloatToQFormatInteger(q1,1,14),
            FloatToQFormatInteger(q2,1,14));
}

int imu_send_massage(float *data, int type)
{
    if(type == 1){
        Data.pSensor->accel[0] = data[0];
        Data.pSensor->accel[1] = data[1];
        Data.pSensor->accel[2] = data[2];
    }
    else if(type == 4){
        Data.pSensor->gyro[0] = data[0];
        Data.pSensor->gyro[1] = data[1];
        Data.pSensor->gyro[2] = data[2];
    }
    else{
        return -3;
    }
    if(imuMsgQueueHandle == NULL) return -2;
    if(xQueueSend(imuMsgQueueHandle, &Data, portMAX_DELAY) != pdPASS){
        return -1;
    }
    return 0;
}
static void imu_thread_func(void const * argument)
{
    imudata mData;
    for(;;){
        if(imuMsgQueueHandle == NULL) continue;
        xQueueReceive( imuMsgQueueHandle, &mData, portMAX_DELAY );
        IMUupdate();
        //imu_get_euler_angles(Data.q);
    }
}
int imu_init(void)
{
    osThreadDef(imu_thread, imu_thread_func,
                osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
    imu_thread_handle = osThreadCreate(osThread(imu_thread), NULL);
    osMessageQDef(imuMsgQueue, 4, imudata);
    imuMsgQueueHandle = osMessageCreate(osMessageQ(imuMsgQueue), NULL);
    return 0;
}
