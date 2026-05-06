#include "imu.h"
#include "imu_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* ─── ICM-45686 via SPI ──────────────────────────────────────────── */
/* Data registers 0x00-0x0B: LE layout — X0(LSB) then X1(MSB) per axis */
#define ICM_REG_ACCEL_X0    0x00  /* burst 12 bytes: ax ay az gx gy gz, little-endian */
#define ICM_REG_PWR_MGMT0   0x10
#define ICM_REG_ACCEL_CFG0  0x1B
#define ICM_REG_GYRO_CFG0   0x1C
#define ICM_REG_WHO_AM_I    0x72
#define ICM_WHOAMI_VAL      0xE9

/* SPI: bit7=1 → read, bit7=0 → write */
#define ICM_SPI_READ        0x80

/* PWR_MGMT0: gyro_mode[3:2]=LN(0b11), accel_mode[1:0]=LN(0b11) */
#define ICM_PWR_LN          0x0F
/* ACCEL_CONFIG0: FS=±4g  (bits[7:5]=0b010→0x40), ODR=200Hz (bits[3:0]=0x09) */
#define ICM_ACCEL_CFG_VAL   (0x40 | 0x09)
/* GYRO_CONFIG0:  FS=±2000dps (bits[7:4]=0b0000→0x00), ODR=200Hz (bits[3:0]=0x09) */
#define ICM_GYRO_CFG_VAL    (0x00 | 0x09)

/* ±4g: 4×9.81/32768 m/s²/LSB */
#define ICM_ACCEL_SCALE     (4.0f * 9.81f / 32768.0f)
/* ±2000dps: 2000×π/(180×32768) rad/s/LSB */
#define ICM_GYRO_SCALE      (2000.0f * 3.14159265f / (180.0f * 32768.0f))

/* CS: P0.4 — managed manually to avoid static-init issues with spi_config.cs */
#define ICM_CS_PIN 4
static const struct device *cs_gpio_dev;
static const struct device *spi_dev;

static const struct spi_config spi_cfg_m0 = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER, /* Mode 0: CPOL=0 CPHA=0 */
	.slave = 0,
};
static const struct spi_config spi_cfg_m3 = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
};
static const struct spi_config *spi_cfg = &spi_cfg_m0;

static inline void cs_low(void)  { gpio_pin_set_raw(cs_gpio_dev, ICM_CS_PIN, 0); }
static inline void cs_high(void) { gpio_pin_set_raw(cs_gpio_dev, ICM_CS_PIN, 1); }

static int icm_spi_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
	/* TX: [read_cmd, dummy×len]  RX: [echo, data×len] — same length for nRF SPIM */
	uint8_t tx[13] = {ICM_SPI_READ | reg};  /* rest are 0x00 */
	uint8_t rx[13];

	if (len > 12) {
		return -EINVAL;
	}

	const struct spi_buf tx_bufs[] = {{.buf = tx, .len = len + 1}};
	const struct spi_buf rx_bufs[] = {{.buf = rx, .len = len + 1}};
	const struct spi_buf_set tx_set = {.buffers = tx_bufs, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = rx_bufs, .count = 1};

	cs_low();
	k_busy_wait(2); /* 2 µs CS setup time */
	int rc = spi_transceive(spi_dev, spi_cfg, &tx_set, &rx_set);
	cs_high();

	if (rc == 0) {
		memcpy(buf, rx + 1, len);
	}
	return rc;
}

static int icm_spi_write(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = {reg & ~ICM_SPI_READ, val};
	const struct spi_buf tx_bufs[] = {{.buf = tx, .len = 2}};
	const struct spi_buf_set tx_set = {.buffers = tx_bufs, .count = 1};

	cs_low();
	k_busy_wait(2);
	int rc = spi_write(spi_dev, spi_cfg, &tx_set);
	cs_high();
	return rc;
}

static bool icm45686_init(void)
{
	spi_cfg = &spi_cfg_m3;
	cs_high();
	k_sleep(K_MSEC(10));

	/* Soft-reset: DEVICE_CONFIG reg 0x01, bit0=1. Required before SPI is stable. */
	icm_spi_write(0x01, 0x01);
	k_sleep(K_MSEC(5));

	/*
	 * WHO_AM_I diagnostic: print raw rx[0] and rx[1] to determine
	 * whether the ICM responds and which byte carries the data.
	 */
	uint8_t tx3[3] = {ICM_SPI_READ | ICM_REG_WHO_AM_I, 0, 0};
	uint8_t rx3[3] = {0};
	const struct spi_buf txb = {.buf = tx3, .len = 3};
	const struct spi_buf rxb = {.buf = rx3, .len = 3};
	const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
	const struct spi_buf_set rxs = {.buffers = &rxb, .count = 1};
	cs_low(); k_busy_wait(2);
	spi_transceive(spi_dev, spi_cfg, &txs, &rxs);
	cs_high();
	printk("ICM WHO_AM_I raw: rx[0]=0x%02X rx[1]=0x%02X (expect 0xE9 somewhere)\n",
	       rx3[0], rx3[1]);

	/* Configure regardless of WHO_AM_I — let data quality confirm the sensor */
	icm_spi_write(ICM_REG_PWR_MGMT0,  0x00);              /* all off */
	k_sleep(K_MSEC(1));
	icm_spi_write(ICM_REG_ACCEL_CFG0, ICM_ACCEL_CFG_VAL); /* ±4g, 200Hz */
	icm_spi_write(ICM_REG_GYRO_CFG0,  ICM_GYRO_CFG_VAL);  /* ±2000dps, 200Hz */
	icm_spi_write(ICM_REG_PWR_MGMT0,  ICM_PWR_LN);        /* both LN */
	k_sleep(K_MSEC(50));

	printk("ICM-45686: configured (Mode3, ±4g, ±2000dps)\n");
	return true;
}

static void icm45686_read(ImuData *d)
{
	uint8_t buf[12];
	if (icm_spi_read(ICM_REG_ACCEL_X0, buf, 12) != 0) {
		return;
	}
	/* little-endian: buf[N]=LSB, buf[N+1]=MSB — ax ay az gx gy gz */
	int16_t ax = (int16_t)((buf[1]  << 8) | buf[0]);
	int16_t ay = (int16_t)((buf[3]  << 8) | buf[2]);
	int16_t az = (int16_t)((buf[5]  << 8) | buf[4]);
	int16_t gx = (int16_t)((buf[7]  << 8) | buf[6]);
	int16_t gy = (int16_t)((buf[9]  << 8) | buf[8]);
	int16_t gz = (int16_t)((buf[11] << 8) | buf[10]);

	d->ax = ax * ICM_ACCEL_SCALE;
	d->ay = ay * ICM_ACCEL_SCALE;
	d->az = az * ICM_ACCEL_SCALE;
	d->gx = gx * ICM_GYRO_SCALE;
	d->gy = gy * ICM_GYRO_SCALE;
	d->gz = gz * ICM_GYRO_SCALE;
	d->timestamp_us = (uint32_t)(k_uptime_get() * 1000U);
}

/* ─── QMC6309 via I2C0 ───────────────────────────────────────────── */
#define QMC_ADDR            0x7C
#define QMC_REG_CHIP_ID     0x00
#define QMC_REG_DATA_X0     0x01  /* 6 bytes little-endian: X Y Z */
#define QMC_REG_STATUS      0x09  /* bit 0 = DRDY */
#define QMC_REG_CTRL1       0x0A
#define QMC_REG_CTRL2       0x0B
#define QMC_CHIP_ID_VAL     0x90
/* CTRL1: mode=continuous(0b11), OSR=512(0b00), LPF=off(0b00) */
#define QMC_CTRL1_VAL       0x03
/* CTRL2: ODR=100Hz(0b10 in [7:6]), RNG=8G(0b10 in [3:2]) */
#define QMC_CTRL2_VAL       0x88
/* ±8G: 1/4000 Gauss/LSB × 100 µT/Gauss */
#define QMC_MAG_SCALE       (100.0f / 4000.0f)

static const struct device *i2c_dev;

static int qmc_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
	return i2c_write_read(i2c_dev, QMC_ADDR, &reg, 1, buf, len);
}

static int qmc_write(uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};
	return i2c_write(i2c_dev, buf, sizeof(buf), QMC_ADDR);
}

static bool qmc6309_init(void)
{
	uint8_t id = 0;
	if (qmc_read(QMC_REG_CHIP_ID, &id, 1) != 0 || id != QMC_CHIP_ID_VAL) {
		printk("QMC6309: not found (chip_id=0x%02X)\n", id);
		return false;
	}
	printk("QMC6309: found\n");

	qmc_write(QMC_REG_CTRL2, QMC_CTRL2_VAL); /* ODR 100Hz, ±8G */
	qmc_write(QMC_REG_CTRL1, QMC_CTRL1_VAL); /* continuous mode */
	k_sleep(K_MSEC(10));

	printk("QMC6309: OK\n");
	return true;
}

static void qmc6309_read(ImuData *d)
{
	uint8_t status = 0;
	if (qmc_read(QMC_REG_STATUS, &status, 1) != 0 || !(status & 0x01)) {
		return;
	}
	uint8_t buf[6];
	if (qmc_read(QMC_REG_DATA_X0, buf, sizeof(buf)) != 0) {
		return;
	}
	/* little-endian */
	int16_t mx = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t my = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t mz = (int16_t)((buf[5] << 8) | buf[4]);

	d->mx = mx * QMC_MAG_SCALE;
	d->my = my * QMC_MAG_SCALE;
	d->mz = mz * QMC_MAG_SCALE;
}

/* ─── sensor thread ──────────────────────────────────────────────── */
static ImuData latest;
static bool data_ready;
static K_MUTEX_DEFINE(dtqsys_mutex);

static void dtqsys_thread(void *p1, void *p2, void *p3)
{
	k_sleep(K_MSEC(1500)); /* wait for USB CDC */

	/* SPI2 for ICM-45686 */
	spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
	if (!device_is_ready(spi_dev)) {
		printk("DTQSYS: spi2 not ready\n");
		return;
	}
	cs_gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	gpio_pin_configure(cs_gpio_dev, ICM_CS_PIN, GPIO_OUTPUT_HIGH); /* CS deasserted */

	/* I2C0 for QMC6309 */
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c_dev)) {
		printk("DTQSYS: i2c0 not ready\n");
		return;
	}

	/* Scan 0x08-0x7F — QMC6309 is at 0x7C which is above the standard 0x77 limit */
	printk("I2C scan 0x08-0x7F:\n");
	int found = 0;
	for (uint8_t addr = 0x08; addr <= 0x7F; addr++) {
		struct i2c_msg msg = {.buf = NULL, .len = 0, .flags = I2C_MSG_WRITE | I2C_MSG_STOP};
		if (i2c_transfer(i2c_dev, &msg, 1, addr) == 0) {
			printk("  found 0x%02X\n", addr);
			found++;
		}
	}
	if (!found) {
		printk("  nothing found\n");
	}

	bool icm_ok = icm45686_init();
	bool qmc_ok = qmc6309_init();

	printk("DTQSYS: init done icm=%d qmc=%d\n", icm_ok, qmc_ok);

	uint32_t tick = 0;
	while (1) {
		if (icm_ok) {
			ImuData d = {0};
			icm45686_read(&d);
			if (qmc_ok) {
				qmc6309_read(&d);
			}
			k_mutex_lock(&dtqsys_mutex, K_FOREVER);
			latest = d;
			data_ready = true;
			k_mutex_unlock(&dtqsys_mutex);
		} else if (++tick % 500 == 0) {
			printk("DTQSYS: ICM not found, retrying WHO_AM_I...\n");
			icm_ok = icm45686_init();
		}
		k_sleep(K_MSEC(10));
	}
}

K_THREAD_DEFINE(dtqsys_tid, 2048, dtqsys_thread, NULL, NULL, NULL, 5, 0, 0);

/* ─── ImuDriverOps ───────────────────────────────────────────────── */
static bool dtqsys_get_data(ImuData *out)
{
	k_mutex_lock(&dtqsys_mutex, K_FOREVER);
	bool ready = data_ready;
	if (ready) {
		*out = latest;
		data_ready = false;
	}
	k_mutex_unlock(&dtqsys_mutex);
	return ready;
}

static const ImuDriverOps dtqsys_driver = {
	.init     = NULL,
	.get_data = dtqsys_get_data,
};

static int dtqsys_register(void)
{
	imu_register_driver(&dtqsys_driver);
	return 0;
}

SYS_INIT(dtqsys_register, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
