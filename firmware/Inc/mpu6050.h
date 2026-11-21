#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <math.h>

/* ── Địa chỉ I²C ─────────────────────────────────────────── */
#define MPU6050_ADDR        (0x68 << 1)

/* ── Thanh ghi ───────────────────────────────────────────── */
#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B

/* ── Hệ số chuyển đổi ────────────────────────────────────── */
#define ACCEL_SCALE         16384.0f   /* ±2g  → LSB/g  */
#define GYRO_SCALE          131.0f     /* ±250°/s → LSB/(°/s) */
#define RAD_TO_DEG          57.2957795f

/* ── Tham số bộ lọc ─────────────────────────────────────── */
#define FILTER_ALPHA        0.96f      /* Tỉ lệ tin tưởng gyro */
#define GYRO_DEADZONE       0.05f      /* °/s — loại drift nhỏ */
#define CALIB_SAMPLES       500        /* Số mẫu calibration    */
#define CALIB_DELAY_MS      2          /* Delay giữa các mẫu   */

/* ── Cấu hình hướng chip ─────────────────────────────────
 * Thay đổi các macro này nếu chip gắn theo hướng khác.
 * Mặc định: chip xoay -90° quanh trục Y
 *   → ax_vật_lý =  az_chip
 *   → ay_vật_lý =  ay_chip
 *   → az_vật_lý = -ax_chip
 * ────────────────────────────────────────────────────── */
#define PHYS_AX(d)  ( (d).raw_ay)   /* Trục nằm ngang phía trước */
#define PHYS_AY(d)  ( (d).raw_az)   /* Trục nằm ngang sang ngang */
#define PHYS_AZ(d)  ( (d).raw_ax)   /* Trục thẳng đứng = trọng lực */
#define PHYS_GX(d)  ( (d).raw_gy)
#define PHYS_GY(d)  ( (d).raw_gz)
#define PHYS_GZ(d)  ( (d).raw_gx)

/* ── Kiểu dữ liệu ────────────────────────────────────────── */
typedef struct {
    /* Raw sau khi chia scale (chưa trừ offset) */
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;

    /* Offset calibration (theo trục vật lý) */
    float off_ax, off_ay, off_az;
    float off_gx, off_gy, off_gz;

    /* Góc đầu ra (độ) */
    float angle_x;   /* Roll  */
    float angle_y;   /* Pitch */
    float angle_z;   /* Yaw — tích phân gyro, có drift dài hạn */
} MPU6050_t;

/* ── API công khai ───────────────────────────────────────── */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
void              MPU6050_Calibrate(void);
void              MPU6050_InitAngles(void);
void              MPU6050_Update(float dt);

/* Con trỏ tới dữ liệu (extern để main.c truy cập) */
extern MPU6050_t MPU6050;

#endif /* MPU6050_H */
