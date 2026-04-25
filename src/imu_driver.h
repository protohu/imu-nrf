#pragma once
#include "imu.h"

typedef struct {
    void (*init)(void);
    bool (*get_data)(ImuData *out);
} ImuDriverOps;

void imu_register_driver(const ImuDriverOps *ops);
