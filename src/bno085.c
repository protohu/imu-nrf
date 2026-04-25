#include "bno085.h"
#include "bno085_hal.h"
#include "imu.h"
#include "imu_driver.h"
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_err.h"
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <string.h>

static ImuData latest;
static bool data_ready;
static K_MUTEX_DEFINE(bno085_mutex);

static void sensor_cb(void *cookie, sh2_SensorEvent_t *event)
{
	sh2_SensorValue_t val;
	if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
		return;
	}

	k_mutex_lock(&bno085_mutex, K_FOREVER);
	switch (val.sensorId) {
	case SH2_GYROSCOPE_CALIBRATED:
		latest.gx = val.un.gyroscope.x;
		latest.gy = val.un.gyroscope.y;
		latest.gz = val.un.gyroscope.z;
		break;
	case SH2_ACCELEROMETER:
		latest.ax = val.un.accelerometer.x;
		latest.ay = val.un.accelerometer.y;
		latest.az = val.un.accelerometer.z;
		latest.timestamp_us = (uint32_t)(k_uptime_get() * 1000U);
		data_ready = true;
		break;
	case SH2_MAGNETIC_FIELD_CALIBRATED:
		latest.mx = val.un.magneticField.x;
		latest.my = val.un.magneticField.y;
		latest.mz = val.un.magneticField.z;
		break;
	}
	k_mutex_unlock(&bno085_mutex);
}

static void configure_sensors(void)
{
	sh2_SensorConfig_t cfg = {0};
	cfg.reportInterval_us = 1000000U / IMU_SAMPLE_HZ;
	sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
	sh2_setSensorConfig(SH2_ACCELEROMETER,        &cfg);
	cfg.reportInterval_us = MAX(1000000U / IMU_SAMPLE_HZ, 10000U);
	sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED, &cfg);
	printk("BNO085: gyro/accel %uHz, mag %uHz\n",
	       IMU_SAMPLE_HZ, 1000000U / cfg.reportInterval_us);
}

static void event_cb(void *cookie, sh2_AsyncEvent_t *event)
{
	if (event->eventId == SH2_RESET) {
		printk("BNO085: reset — reconfiguring sensors\n");
		configure_sensors();
	}
}

static void bno085_thread(void *p1, void *p2, void *p3)
{
	k_sleep(K_SECONDS(5));
	printk("BNO085: thread started, opening...\n");
	int ret = sh2_open(&bno085_hal, event_cb, NULL);
	if (ret != SH2_OK) {
		printk("BNO085: sh2_open failed: %d\n", ret);
		return;
	}
	printk("BNO085: sh2_open OK\n");

	sh2_setSensorCallback(sensor_cb, NULL);

	k_sleep(K_MSEC(100));

	configure_sensors();

	while (1) {
		sh2_service();
		k_yield();
	}
}

K_THREAD_DEFINE(bno085_tid, 4096, bno085_thread, NULL, NULL, NULL, 5, 0, 0);

static bool bno085_get_data(ImuData *out)
{
	k_mutex_lock(&bno085_mutex, K_FOREVER);
	bool ready = data_ready;
	if (ready) {
		*out = latest;
		data_ready = false;
	}
	k_mutex_unlock(&bno085_mutex);
	return ready;
}

static const ImuDriverOps bno085_driver = {
	.init     = NULL,
	.get_data = bno085_get_data,
};

static int bno085_register(void)
{
	imu_register_driver(&bno085_driver);
	return 0;
}

SYS_INIT(bno085_register, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
