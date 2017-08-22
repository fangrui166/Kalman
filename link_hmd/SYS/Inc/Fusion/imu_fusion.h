typedef struct {
    // quaterions w,x,y,z
    int16_t             q_w;
    int16_t             q_x;
    int16_t             q_y;
    int16_t             q_z;
    // acceleration x,y,z
    int16_t             a_x;
    int16_t             a_y;
    int16_t             a_z;
    // qyro x,y,z
    int16_t             g_x;
    int16_t             g_y;
    int16_t             g_z;
    // gyro angle acc x,y,z
    int16_t             g_angleacc_x;
    int16_t             g_angleacc_y;
    int16_t             g_angleacc_z;
    int16_t             padding;  // just for struct 4 bytes alignmnet

    long                serial;
    long long           timeStamp;
} imu_t;

