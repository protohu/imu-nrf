#include "imu.h"
#include "imu_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* ── ITG3200 gyroscope ───────────────────────────────────────────────────── */
#define ITG_ADDR       0x68u
#define ITG_WHO_AM_I   0x00u
#define ITG_SMPLRT_DIV 0x15u
#define ITG_DLPF_FS    0x16u
#define ITG_PWR_MGM    0x3Eu
#define ITG_TEMP_H     0x1Bu  /* TEMP_H TEMP_L GX_H GX_L GY_H GY_L GZ_H GZ_L */
/* FS_SEL=3 → 14.375 LSB/°/s; to rad/s: raw × π/(14.375×180) */
#define ITG_SCALE      (3.14159265f / (14.375f * 180.0f))

/* ── ADXL345 accelerometer ───────────────────────────────────────────────── */
#define ADXL_ADDR      0x53u  /* SDO=GND on standard GY-85; use 0x1D if SDO=3V3 */
#define ADXL_DEVID     0x00u
#define ADXL_BW_RATE   0x2Cu
#define ADXL_PWR_CTL   0x2Du
#define ADXL_FORMAT    0x31u
#define ADXL_DATAX0    0x32u
/* ±2g, 256 LSB/g; to m/s²: raw × 9.81/256 */
#define ADXL_SCALE     (9.81f / 256.0f)

/* ── HMC5883L magnetometer ───────────────────────────────────────────────── */
#define HMC_ADDR       0x1Eu
#define HMC_CFG_A      0x00u
#define HMC_CFG_B      0x01u
#define HMC_MODE       0x02u
#define HMC_DATA_H     0x03u  /* X_H X_L Z_H Z_L Y_H Y_L (Z before Y!) */
#define HMC_STATUS     0x09u
#define HMC_IDA        0x0Au  /* 'H' = 0x48 */
/* gain=1 (Config B=0x20) → 1090 LSB/Ga = 10.9 LSB/µT; to µT: raw/10.9 */
#define HMC_SCALE      (1.0f / 10.9f)
#define HMC_POLL_MS    14u    /* 75 Hz output → poll every ~13 ms */

static const struct device *i2c_dev;

static inline int rd(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	return i2c_burst_read(i2c_dev, addr, reg, buf, len);
}

static inline int wr(uint8_t addr, uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte(i2c_dev, addr, reg, val);
}

/* ── ITG3200 ─────────────────────────────────────────────────────────────── */
static bool itg3200_init(void)
{
	uint8_t id = 0;
	if (rd(ITG_ADDR, ITG_WHO_AM_I, &id, 1) != 0) {
		printk("ITG3200: no ACK at 0x%02X\n", ITG_ADDR);
		return false;
	}
	/* WHO_AM_I bits[6:1] = fixed 110100; bit0 = AD0. Expect 0x68 or 0x69 */
	if (id != 0x68 && id != 0x69) {
		printk("ITG3200 WHO_AM_I=0x%02X unexpected\n", id);
		return false;
	}
	printk("ITG3200 WHO_AM_I=0x%02X OK\n", id);

	wr(ITG_ADDR, ITG_PWR_MGM, 0x80);  /* reset */
	k_sleep(K_MSEC(10));
	wr(ITG_ADDR, ITG_DLPF_FS, 0x19);  /* FS_SEL=3 (±2000°/s), DLPF=1 (188Hz BW) */
	wr(ITG_ADDR, ITG_SMPLRT_DIV, 9);  /* 1kHz / 10 = 100Hz sample rate */
	wr(ITG_ADDR, ITG_PWR_MGM, 0x01);  /* PLL with X gyro clock */
	k_sleep(K_MSEC(50));
	return true;
}

static bool itg3200_read(float *gx, float *gy, float *gz)
{
	uint8_t buf[8];
	if (rd(ITG_ADDR, ITG_TEMP_H, buf, 8) != 0) return false;
	/* big-endian: [TEMP_H TEMP_L GX_H GX_L GY_H GY_L GZ_H GZ_L] */
	int16_t rx = (int16_t)((buf[2] << 8) | buf[3]);
	int16_t ry = (int16_t)((buf[4] << 8) | buf[5]);
	int16_t rz = (int16_t)((buf[6] << 8) | buf[7]);
	*gx = rx * ITG_SCALE;
	*gy = ry * ITG_SCALE;
	*gz = rz * ITG_SCALE;
	return true;
}

/* ── ADXL345 ─────────────────────────────────────────────────────────────── */
static bool adxl345_init(void)
{
	uint8_t id = 0;
	if (rd(ADXL_ADDR, ADXL_DEVID, &id, 1) != 0) {
		printk("ADXL345: no ACK at 0x%02X\n", ADXL_ADDR);
		return false;
	}
	if (id != 0xE5) {
		printk("ADXL345 DEVID=0x%02X unexpected (expect 0xE5)\n", id);
		return false;
	}
	printk("ADXL345 DEVID=0xE5 OK\n");

	wr(ADXL_ADDR, ADXL_FORMAT,  0x00);  /* ±2g, right-justified 10-bit */
	wr(ADXL_ADDR, ADXL_BW_RATE, 0x0A); /* 100Hz output */
	wr(ADXL_ADDR, ADXL_PWR_CTL, 0x08); /* measure mode */
	k_sleep(K_MSEC(10));
	return true;
}

static bool adxl345_read(float *ax, float *ay, float *az)
{
	uint8_t buf[6];
	if (rd(ADXL_ADDR, ADXL_DATAX0, buf, 6) != 0) return false;
	/* little-endian: [X0 X1 Y0 Y1 Z0 Z1] */
	int16_t rx = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t ry = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t rz = (int16_t)((buf[5] << 8) | buf[4]);
	*ax = rx * ADXL_SCALE;
	*ay = ry * ADXL_SCALE;
	*az = rz * ADXL_SCALE;
	return true;
}

/* ── HMC5883L ────────────────────────────────────────────────────────────── */
static float hmc_last[3] = {0};
static uint32_t hmc_next_ms = 0;

static bool hmc5883l_init(void)
{
	uint8_t id = 0;
	if (rd(HMC_ADDR, HMC_IDA, &id, 1) != 0) {
		printk("HMC5883L: no ACK at 0x%02X\n", HMC_ADDR);
		return false;
	}
	if (id != 0x48) {  /* 'H' */
		printk("HMC5883L IDA=0x%02X unexpected (expect 0x48)\n", id);
		return false;
	}
	printk("HMC5883L IDA=0x48 OK\n");

	wr(HMC_ADDR, HMC_CFG_A, 0x18);  /* 1-sample avg, 75Hz output rate */
	wr(HMC_ADDR, HMC_CFG_B, 0x20);  /* gain=1: ±1.3 Ga, 1090 LSB/Ga */
	wr(HMC_ADDR, HMC_MODE,  0x00);  /* continuous measurement mode */
	k_sleep(K_MSEC(10));
	return true;
}

static void hmc5883l_update(float *mx, float *my, float *mz)
{
	uint32_t now = k_uptime_get_32();
	if ((int32_t)(now - hmc_next_ms) < 0) {
		*mx = hmc_last[0]; *my = hmc_last[1]; *mz = hmc_last[2];
		return;
	}
	hmc_next_ms = now + HMC_POLL_MS;

	uint8_t st = 0;
	if (rd(HMC_ADDR, HMC_STATUS, &st, 1) != 0 || !(st & 0x01)) {
		*mx = hmc_last[0]; *my = hmc_last[1]; *mz = hmc_last[2];
		return;
	}

	uint8_t buf[6];
	if (rd(HMC_ADDR, HMC_DATA_H, buf, 6) != 0) {
		*mx = hmc_last[0]; *my = hmc_last[1]; *mz = hmc_last[2];
		return;
	}

	/* big-endian: [X_H X_L Z_H Z_L Y_H Y_L] — Z before Y is correct per datasheet */
	int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
	int16_t rz = (int16_t)((buf[2] << 8) | buf[3]);
	int16_t ry = (int16_t)((buf[4] << 8) | buf[5]);

	/* ±4096 = overflow sentinel */
	if (rx == -4096 || ry == -4096 || rz == -4096) {
		*mx = hmc_last[0]; *my = hmc_last[1]; *mz = hmc_last[2];
		return;
	}

	hmc_last[0] = rx * HMC_SCALE;
	hmc_last[1] = ry * HMC_SCALE;
	hmc_last[2] = rz * HMC_SCALE;
	*mx = hmc_last[0]; *my = hmc_last[1]; *mz = hmc_last[2];
}

/* ── Sensor thread ───────────────────────────────────────────────────────── */
static ImuData latest = {0};
static bool data_ready = false;
static K_MUTEX_DEFINE(gy85_mutex);

static void gy85_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	k_sleep(K_MSEC(4000));

	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		printk("I2C0 not ready! Check app.overlay\n");
		return;
	}

	bool itg_ok = false, adxl_ok = false, hmc_ok = false;
	for (int attempt = 1; !(itg_ok && adxl_ok); attempt++) {
		itg_ok  = itg3200_init();
		adxl_ok = adxl345_init();
		if (!(itg_ok && adxl_ok)) {
			printk("GY-85 init fail (attempt %d), retry in 2s\n", attempt);
			k_sleep(K_MSEC(2000));
		}
	}
	hmc_ok = hmc5883l_init();
	printk("GY-85 ready: ITG=%d ADXL=%d HMC=%d\n", itg_ok, adxl_ok, hmc_ok);

	int64_t next_tick = k_uptime_get() + 10;

	while (1) {
		float gx, gy, gz, ax, ay, az, mx = 0, my = 0, mz = 0;

		bool ok = itg3200_read(&gx, &gy, &gz) && adxl345_read(&ax, &ay, &az);
		if (hmc_ok) hmc5883l_update(&mx, &my, &mz);

		if (ok) {
			k_mutex_lock(&gy85_mutex, K_FOREVER);
			latest.gx = gx; latest.gy = gy; latest.gz = gz;
			latest.ax = ax; latest.ay = ay; latest.az = az;
			latest.mx = mx; latest.my = my; latest.mz = mz;
			latest.timestamp_us = (uint32_t)(k_uptime_get() * 1000U);
			data_ready = true;
			k_mutex_unlock(&gy85_mutex);
		}

		int64_t delay = next_tick - k_uptime_get();
		if (delay > 0) k_sleep(K_MSEC((int32_t)delay));
		next_tick += 10;
	}
}

K_THREAD_DEFINE(gy85_tid, 2048, gy85_thread, NULL, NULL, NULL, 5, 0, 0);

/* ── IMU driver registration ─────────────────────────────────────────────── */
static bool gy85_get_data(ImuData *out)
{
	k_mutex_lock(&gy85_mutex, K_FOREVER);
	bool ready = data_ready;
	if (ready) {
		*out = latest;
		data_ready = false;
	}
	k_mutex_unlock(&gy85_mutex);
	return ready;
}

static const ImuDriverOps gy85_driver = {
	.init     = NULL,
	.get_data = gy85_get_data,
};

static int gy85_register(void)
{
	imu_register_driver(&gy85_driver);
	return 0;
}

SYS_INIT(gy85_register, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
