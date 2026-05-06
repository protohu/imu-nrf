#include "imu.h"
#include "imu_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* ─── ICM-45686 via SPI3 ─────────────────────────────────────────── */
/* Registers (direct register map) */
#define ICM_REG_ACCEL_X1    0x00  /* burst 12 bytes: ax ay az gx gy gz, big-endian */
#define ICM_REG_PWR_MGMT0   0x10
#define ICM_REG_ACCEL_CFG0  0x1B
#define ICM_REG_GYRO_CFG0   0x1C
#define ICM_REG_WHO_AM_I    0x72
#define ICM_WHOAMI_VAL      0xE9

/* SPI: bit7=1 → read, bit7=0 → write */
#define ICM_SPI_READ        0x80

/* PWR_MGMT0: gyro_mode[3:2]=LN(0b11), accel_mode[1:0]=LN(0b11) */
#define ICM_PWR_LN          0x0F
/* ACCEL_CONFIG0: FS=±16G (bits[6:4]=0x1), ODR=100Hz (bits[3:0]=0x9) */
#define ICM_ACCEL_CFG_VAL   ((0x1 << 4) | 0x9)
/* GYRO_CONFIG0:  FS=±2000dps (bits[7:4]=0x1), ODR=100Hz (bits[3:0]=0x9) */
#define ICM_GYRO_CFG_VAL    ((0x1 << 4) | 0x9)

/* ±16G: 16/32768 g/LSB × 9.81 m/s²/g */
#define ICM_ACCEL_SCALE     (16.0f * 9.81f / 32768.0f)
/* ±2000dps: 2000/32768 × π/180 rad/s/LSB */
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
	cs_high();
	k_sleep(K_MSEC(10)); /* CS high: reset ICM SPI state machine */

	/* sweep Mode 3 — ICM responds to Mode 3 clock */
	spi_cfg = &spi_cfg_m3;
	printk("ICM Mode3 regs 0x00-0x7F:\n");
	for (uint8_t r = 0; r <= 0x7F; r++) {
		uint8_t v = 0;
		icm_spi_read(r, &v, 1);
		printk(" %02X", v);
		if ((r & 0x0F) == 0x0F) printk("\n");
	}
	printk("\n");

	/* sweep Mode 0 for comparison */
	cs_high();
	k_sleep(K_MSEC(10));
	spi_cfg = &spi_cfg_m0;
	printk("ICM Mode0 regs 0x70-0x7F:");
	for (uint8_t r = 0x70; r <= 0x7F; r++) {
		uint8_t v = 0;
		icm_spi_read(r, &v, 1);
		printk(" %02X", v);
	}
	printk("\n");

	/* check WHO_AM_I in both modes */
	uint8_t id = 0;
	spi_cfg = &spi_cfg_m3;
	k_sleep(K_MSEC(5));
	icm_spi_read(ICM_REG_WHO_AM_I, &id, 1);
	if (id != ICM_WHOAMI_VAL) {
		spi_cfg = &spi_cfg_m0;
		k_sleep(K_MSEC(5));
		icm_spi_read(ICM_REG_WHO_AM_I, &id, 1);
	}
	if (id != ICM_WHOAMI_VAL) {
		printk("ICM-45686: not found (0x%02X), expected 0x%02X\n", id, ICM_WHOAMI_VAL);
		spi_cfg = &spi_cfg_m0;
		return false;
	}
	printk("ICM-45686: found, WHO_AM_I=0x%02X\n", id);

	icm_spi_write(ICM_REG_PWR_MGMT0,  0x00);              /* all off */
	k_sleep(K_MSEC(1));
	icm_spi_write(ICM_REG_ACCEL_CFG0, ICM_ACCEL_CFG_VAL); /* ±16G, 100Hz */
	icm_spi_write(ICM_REG_GYRO_CFG0,  ICM_GYRO_CFG_VAL);  /* ±2000dps, 100Hz */
	icm_spi_write(ICM_REG_PWR_MGMT0,  ICM_PWR_LN);        /* both LN */
	k_sleep(K_MSEC(50)); /* gyro LN startup: ~30ms */

	printk("ICM-45686: OK\n");
	return true;
}

static void icm45686_read(ImuData *d)
{
	uint8_t buf[12];
	if (icm_spi_read(ICM_REG_ACCEL_X1, buf, 12) != 0) {
		return;
	}
	/* big-endian int16: ax ay az gx gy gz */
	int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
	int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
	int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
	int16_t gx = (int16_t)((buf[6]  << 8) | buf[7]);
	int16_t gy = (int16_t)((buf[8]  << 8) | buf[9]);
	int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);

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

	/* Scan all I2C addresses to find what's on the bus */
	printk("I2C scan:\n");
	int found = 0;
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		struct i2c_msg msg = {.buf = NULL, .len = 0, .flags = I2C_MSG_WRITE | I2C_MSG_STOP};
		if (i2c_transfer(i2c_dev, &msg, 1, addr) == 0) {
			printk("  found device at 0x%02X\n", addr);
			found++;
		}
	}
	if (!found) {
		printk("  no devices found on i2c0\n");
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
