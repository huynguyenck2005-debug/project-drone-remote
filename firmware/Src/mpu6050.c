#include "mpu6050.h"

/* ── Biến nội bộ ─────────────────────────────────────────── */
MPU6050_t MPU6050 = {0};
static I2C_HandleTypeDef *_hi2c = NULL;

/* ══════════════════════════════════════════════════════════
 * Hàm nội bộ
 * ══════════════════════════════════════════════════════════ */

static HAL_StatusTypeDef _write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(_hi2c, MPU6050_ADDR,
                             reg, 1, &val, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef _read_raw(void)
{
    uint8_t buf[14];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(_hi2c, MPU6050_ADDR,
                           REG_ACCEL_XOUT_H, 1,
                           buf, 14, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    MPU6050.raw_ax = (int16_t)(buf[0]  << 8 | buf[1])  / ACCEL_SCALE;
    MPU6050.raw_ay = (int16_t)(buf[2]  << 8 | buf[3])  / ACCEL_SCALE;
    MPU6050.raw_az = (int16_t)(buf[4]  << 8 | buf[5])  / ACCEL_SCALE;
    /* buf[6..7] = nhiệt độ nội, bỏ qua */
    MPU6050.raw_gx = (int16_t)(buf[8]  << 8 | buf[9])  / GYRO_SCALE;
    MPU6050.raw_gy = (int16_t)(buf[10] << 8 | buf[11]) / GYRO_SCALE;
    MPU6050.raw_gz = (int16_t)(buf[12] << 8 | buf[13]) / GYRO_SCALE;

    return HAL_OK;
}

/* Lấy accel/gyro theo trục vật lý (đã remap + trừ offset) */
static void _get_physical(float *ax, float *ay, float *az,
                           float *gx, float *gy, float *gz)
{
    *ax = PHYS_AX(MPU6050) - MPU6050.off_ax;
    *ay = PHYS_AY(MPU6050) - MPU6050.off_ay;
    *az = PHYS_AZ(MPU6050) - MPU6050.off_az;
    *gx = PHYS_GX(MPU6050) - MPU6050.off_gx;
    *gy = PHYS_GY(MPU6050) - MPU6050.off_gy;
    *gz = PHYS_GZ(MPU6050) - MPU6050.off_gz;
}

/* ══════════════════════════════════════════════════════════
 * API công khai
 * ══════════════════════════════════════════════════════════ */

/**
 * @brief Khởi tạo MPU6050: wake up, cấu hình gyro ±250°/s, accel ±2g.
 * @return HAL_OK nếu thành công.
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    HAL_Delay(100);

    HAL_StatusTypeDef ret;
    ret = _write(REG_PWR_MGMT_1,   0x00);  /* Wake up                */
    if (ret != HAL_OK) return ret;
    ret = _write(REG_GYRO_CONFIG,  0x00);  /* Gyro ±250 °/s          */
    if (ret != HAL_OK) return ret;
    ret = _write(REG_ACCEL_CONFIG, 0x00);  /* Accel ±2g              */
    return ret;
}

/**
 * @brief Calibration: lấy trung bình 500 mẫu lúc đứng yên.
 *        Đặt chip nằm phẳng trước khi gọi hàm này.
 */
void MPU6050_Calibrate(void)
{
    float sum[6] = {0};

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        _read_raw();
        sum[0] += PHYS_AX(MPU6050);
        sum[1] += PHYS_AY(MPU6050);
        sum[2] += PHYS_AZ(MPU6050);
        sum[3] += PHYS_GX(MPU6050);
        sum[4] += PHYS_GY(MPU6050);
        sum[5] += PHYS_GZ(MPU6050);
        HAL_Delay(CALIB_DELAY_MS);
    }

    MPU6050.off_ax = sum[0] / CALIB_SAMPLES;
    MPU6050.off_ay = sum[1] / CALIB_SAMPLES;
    MPU6050.off_az = sum[2] / CALIB_SAMPLES - 1.0f; /* Trừ 1g trục Z vật lý */
    MPU6050.off_gx = sum[3] / CALIB_SAMPLES;
    MPU6050.off_gy = sum[4] / CALIB_SAMPLES;
    MPU6050.off_gz = sum[5] / CALIB_SAMPLES;
}

/**
 * @brief Khởi tạo góc ban đầu từ accel (tránh hội tụ chậm).
 *        Gọi sau MPU6050_Calibrate().
 */
void MPU6050_InitAngles(void)
{
    _read_raw();

    float ax, ay, az, gx, gy, gz;
    _get_physical(&ax, &ay, &az, &gx, &gy, &gz);

    MPU6050.angle_x = atan2f(ay, az) * RAD_TO_DEG;
    MPU6050.angle_y = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
    MPU6050.angle_z = 0.0f;  /* Yaw không có reference tuyệt đối */
}

/**
 * @brief Cập nhật góc — gọi mỗi vòng lặp với dt chính xác (giây).
 * @param dt Thời gian từ lần gọi trước (giây).
 */
void MPU6050_Update(float dt)
{
    if (_read_raw() != HAL_OK) return;  /* Bỏ qua nếu I²C lỗi */

    float ax, ay, az, gx, gy, gz;
    _get_physical(&ax, &ay, &az, &gx, &gy, &gz);

    /* Dead-zone gyro Z để giảm drift Yaw lúc đứng yên */
    if (fabsf(gz) < GYRO_DEADZONE) gz = 0.0f;

    /* Góc accel (reference tĩnh cho Roll và Pitch) */
    float acc_x = atan2f(ay, az) * RAD_TO_DEG;
    float acc_y = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

    /* Bộ lọc complementary */
    MPU6050.angle_x = FILTER_ALPHA * (MPU6050.angle_x + gx * dt)
                    + (1.0f - FILTER_ALPHA) * acc_x;
    MPU6050.angle_y = FILTER_ALPHA * (MPU6050.angle_y + gy * dt)
                    + (1.0f - FILTER_ALPHA) * acc_y;
    MPU6050.angle_z += gz * dt;
}
