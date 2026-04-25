#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Master sample rate. Controls sensor config, console log rate, and BLE notify rate. */
#define IMU_SAMPLE_HZ    100U
#define IMU_LOG_HZ       10U   /* console prints per second  = IMU_SAMPLE_HZ / IMU_LOG_HZ    */
#define IMU_BLE_HZ       50U   /* BLE notifications per sec  = IMU_SAMPLE_HZ / IMU_BLE_HZ    */

typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    uint32_t timestamp_us;
} ImuData;

void imu_init(void);
bool imu_get_data(ImuData *out);
