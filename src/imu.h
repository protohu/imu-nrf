#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    uint32_t timestamp_us;
} ImuData;

void imu_init(void);
bool imu_get_data(ImuData *out);
