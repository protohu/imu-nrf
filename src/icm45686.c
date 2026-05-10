#include "imu.h"
#include "imu_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

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
/* ACCEL_CONFIG0: FS=±4g, ODR=200Hz */
#define ICM_ACCEL_CFG_VAL   (0x40 | 0x09)
/* GYRO_CONFIG0: FS=±2000dps, ODR=200Hz */
#define ICM_GYRO_CFG_VAL    (0x00 | 0x09)

/* ±4g: 4×9.81/32768 m/s²/LSB */
#define ICM_ACCEL_SCALE     (4.0f * 9.81f / 32768.0f)
/* ±2000dps: 2000×π/(180×32768) rad/s/LSB */
#define ICM_GYRO_SCALE      (2000.0f * 3.14159265f / (180.0f * 32768.0f))

/* CS: P0.4 — managed manually */
#define ICM_CS_PIN 4

static const struct device *cs_gpio_dev;
static const struct device *spi_dev;

static const struct spi_config spi_cfg_m0 = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
	.slave = 0,
};

static const struct spi_config spi_cfg_m3 = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
};

static const struct spi_config *spi_cfg = &spi_cfg_m0;

static inline void cs_low(void)
{
	gpio_pin_set_raw(cs_gpio_dev, ICM_CS_PIN, 0);
}

static inline void cs_high(void)
{
	gpio_pin_set_raw(cs_gpio_dev, ICM_CS_PIN, 1);
}

static int icm_spi_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
	if (len > 12) {
		return -EINVAL;
	}

	uint8_t tx[13] = { ICM_SPI_READ | reg };
	uint8_t rx[13] = { 0 };

	const struct spi_buf tx_bufs[] = {
		{ .buf = tx, .len = len + 1 },
	};
	const struct spi_buf rx_bufs[] = {
		{ .buf = rx, .len = len + 1 },
	};
	const struct spi_buf_set tx_set = {
		.buffers = tx_bufs,
		.count = 1,
	};
	const struct spi_buf_set rx_set = {
		.buffers = rx_bufs,
		.count = 1,
	};

	cs_low();
	k_busy_wait(2);
	int rc = spi_transceive(spi_dev, spi_cfg, &tx_set, &rx_set);
	cs_high();

	if (rc == 0) {
		memcpy(buf, rx + 1, len);
	}
	return rc;
}

static int icm_spi_write(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = { reg & ~ICM_SPI_READ, val };

	const struct spi_buf tx_bufs[] = {
		{ .buf = tx, .len = 2 },
	};
	const struct spi_buf_set tx_set = {
		.buffers = tx_bufs,
		.count = 1,
	};

	cs_low();
	k_busy_wait(2);
	int rc = spi_write(spi_dev, spi_cfg, &tx_set);
	cs_high();
	return rc;
}

/* ─── ICM-45686 indirect register access ─────────────────────────── */
#define ICM_IREG_ADDR_H  0x7C
#define ICM_IREG_DATA    0x7E

static int icm_mreg_write(uint16_t addr, uint8_t val)
{
	uint8_t tx[4] = { ICM_IREG_ADDR_H, addr >> 8, addr & 0xFF, val };

	const struct spi_buf txb = {
		.buf = tx,
		.len = 4,
	};
	const struct spi_buf_set txs = {
		.buffers = &txb,
		.count = 1,
	};

	cs_low();
	k_busy_wait(4);
	int rc = spi_write(spi_dev, spi_cfg, &txs);
	cs_high();
	k_busy_wait(4);
	return rc;
}

static int icm_mreg_read(uint16_t addr, uint8_t *val)
{
	uint8_t tx[3] = { ICM_IREG_ADDR_H, addr >> 8, addr & 0xFF };

	const struct spi_buf txb = {
		.buf = tx,
		.len = 3,
	};
	const struct spi_buf_set txs = {
		.buffers = &txb,
		.count = 1,
	};

	cs_low();
	k_busy_wait(4);
	int rc = spi_write(spi_dev, spi_cfg, &txs);
	cs_high();
	if (rc != 0) {
		return rc;
	}

	k_busy_wait(4);
	return icm_spi_read(ICM_IREG_DATA, val, 1);
}

static int icm_mreg_read_bytes(uint16_t addr, uint8_t *buf, int n)
{
	for (int i = 0; i < n; i++) {
		int rc = icm_mreg_read(addr + i, &buf[i]);
		if (rc != 0) {
			return rc;
		}
	}
	return 0;
}

/* ─── ICM-45686 I2C master (AUX1) ────────────────────────────────── */
/*
 * ICM I2CM registers live in MREG2:
 *   COMMAND_0      0xa206
 *   DEV_PROFILE0   0xa20e
 *   DEV_PROFILE1   0xa20f
 *   CONTROL        0xa216
 *   STATUS         0xa218
 *   EXT_DEV_STATUS 0xa21a
 *   RD_DATA0       0xa21b
 *   WR_DATA0       0xa233
 */
#define I2CM_COMMAND_0      0xa206u
#define I2CM_DEV_PROFILE0   0xa20eu
#define I2CM_DEV_PROFILE1   0xa20fu
#define I2CM_CONTROL        0xa216u
#define I2CM_STATUS         0xa218u
#define I2CM_EXT_DEV_STATUS 0xa21au
#define I2CM_RD_DATA0       0xa21bu
#define I2CM_WR_DATA0       0xa233u

static uint8_t icm_i2cm_wait(void)
{
	uint8_t st = 0;

	for (int i = 0; i < 200; i++) {
		if (icm_mreg_read(I2CM_STATUS, &st) != 0) {
			break;
		}
		if (st & 0x02) {
			break;
		}
		k_busy_wait(10);
	}
	return st;
}

static int icm_i2cm_write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t val)
{
	if (icm_mreg_write(I2CM_DEV_PROFILE1, i2c_addr) != 0) {
		return -EIO;
	}
	if (icm_mreg_write(I2CM_WR_DATA0, reg) != 0) {
		return -EIO;
	}
	if (icm_mreg_write(I2CM_WR_DATA0 + 1, val) != 0) {
		return -EIO;
	}

	/* write, len=2(reg+data), endflag */
	if (icm_mreg_write(I2CM_COMMAND_0, 0x82) != 0) {
		return -EIO;
	}
	/* Standard mode + GO, no restart needed for pure write */
	if (icm_mreg_write(I2CM_CONTROL, 0x09) != 0) {
		return -EIO;
	}

	uint8_t st = icm_i2cm_wait();
	uint8_t dev_st = 0;
	(void)icm_mreg_read(I2CM_EXT_DEV_STATUS, &dev_st);

	if ((st & 0x3C) != 0 || (dev_st & 0x01) != 0) {
		return -EIO;
	}
	return 0;
}

static int icm_i2cm_read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t *val)
{
	if (icm_mreg_write(I2CM_DEV_PROFILE0, reg) != 0) {
		return -EIO;
	}
	if (icm_mreg_write(I2CM_DEV_PROFILE1, i2c_addr) != 0) {
		return -EIO;
	}
	/* read, reg-address phase, 1 byte */
	if (icm_mreg_write(I2CM_COMMAND_0, 0x91) != 0) {
		return -EIO;
	}
	/* Restart + Standard mode + GO */
	if (icm_mreg_write(I2CM_CONTROL, 0x49) != 0) {
		return -EIO;
	}

	uint8_t st = icm_i2cm_wait();
	uint8_t dev_st = 0;
	(void)icm_mreg_read(I2CM_EXT_DEV_STATUS, &dev_st);

	if ((st & 0x3C) != 0 || (dev_st & 0x01) != 0) {
		return -EIO;
	}

	return icm_mreg_read(I2CM_RD_DATA0, val);
}

static int icm_i2cm_read_bytes(uint8_t i2c_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	if (len == 0 || len > 15) {
		return -EINVAL;
	}

	if (icm_mreg_write(I2CM_DEV_PROFILE0, reg) != 0) {
		return -EIO;
	}
	if (icm_mreg_write(I2CM_DEV_PROFILE1, i2c_addr) != 0) {
		return -EIO;
	}
	/* read, reg-address phase, burst len */
	if (icm_mreg_write(I2CM_COMMAND_0, (uint8_t)(0x90 | (len & 0x0F))) != 0) {
		return -EIO;
	}
	/* Restart + Standard mode + GO */
	if (icm_mreg_write(I2CM_CONTROL, 0x49) != 0) {
		return -EIO;
	}

	uint8_t st = icm_i2cm_wait();
	uint8_t dev_st = 0;
	(void)icm_mreg_read(I2CM_EXT_DEV_STATUS, &dev_st);

	if ((st & 0x3C) != 0 || (dev_st & 0x01) != 0) {
		return -EIO;
	}

	for (int i = 0; i < len; i++) {
		if (icm_mreg_read(I2CM_RD_DATA0 + i, &buf[i]) != 0) {
			return -EIO;
		}
	}
	return 0;
}

/* ─── ICM-45686 init ─────────────────────────────────────────────── */
static bool icm45686_init(void)
{
	spi_cfg = &spi_cfg_m3;
	cs_high();
	k_sleep(K_MSEC(10));

	/* Soft-reset */
	(void)icm_spi_write(0x01, 0x01);
	k_sleep(K_MSEC(5));

	uint8_t who = 0;
	if (icm_spi_read(ICM_REG_WHO_AM_I, &who, 1) == 0) {
		printk("ICM WHO_AM_I = 0x%02X\n", who);
	}

	(void)icm_spi_write(ICM_REG_PWR_MGMT0, 0x00);
	k_sleep(K_MSEC(1));
	(void)icm_spi_write(ICM_REG_ACCEL_CFG0, ICM_ACCEL_CFG_VAL);
	(void)icm_spi_write(ICM_REG_GYRO_CFG0, ICM_GYRO_CFG_VAL);
	(void)icm_spi_write(ICM_REG_PWR_MGMT0, ICM_PWR_LN);
	k_sleep(K_MSEC(50));

	printk("ICM-45686: configured (Mode3, ±4g, ±2000dps)\n");

	/* Enable AUX1 as I2CM master */
	uint8_t ioc_pad = 0;
	if (icm_spi_read(0x2F, &ioc_pad, 1) == 0) {
		printk("ICM-45686: IOC_PAD_SCENARIO before=0x%02X\n", ioc_pad);
		ioc_pad = (ioc_pad & ~0x07u) | 0x03u;
		(void)icm_spi_write(0x2F, ioc_pad);
	}

	uint8_t aux_ovrd = 0;
	if (icm_spi_read(0x30, &aux_ovrd, 1) == 0) {
		aux_ovrd = (aux_ovrd & ~0x1Fu) | 0x17u;
		(void)icm_spi_write(0x30, aux_ovrd);
	}
	k_sleep(K_MSEC(5));

	if (icm_spi_read(0x2F, &ioc_pad, 1) == 0 &&
	    icm_spi_read(0x30, &aux_ovrd, 1) == 0) {
		printk("ICM-45686: IOC_PAD_SCENARIO=0x%02X AUX_OVRD=0x%02X\n",
		       ioc_pad, aux_ovrd);
	}

	return true;
}

/* ─── ICM readout ────────────────────────────────────────────────── */
static void icm45686_read(ImuData *d)
{
	uint8_t buf[12];
	if (icm_spi_read(ICM_REG_ACCEL_X0, buf, 12) != 0) {
		return;
	}

	int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
	int16_t gx = (int16_t)((buf[7] << 8) | buf[6]);
	int16_t gy = (int16_t)((buf[9] << 8) | buf[8]);
	int16_t gz = (int16_t)((buf[11] << 8) | buf[10]);

	d->ax = ax * ICM_ACCEL_SCALE;
	d->ay = ay * ICM_ACCEL_SCALE;
	d->az = az * ICM_ACCEL_SCALE;
	d->gx = gx * ICM_GYRO_SCALE;
	d->gy = gy * ICM_GYRO_SCALE;
	d->gz = gz * ICM_GYRO_SCALE;
	d->timestamp_us = (uint32_t)(k_uptime_get() * 1000U);
}

/* ─── QMC6309 via ICM-45686 I2CM ────────────────────────────────── */
/*
 * QMC6309 datasheet: I2C-only, chip ID at 0x00 = 0x90,
 * data registers 0x01..0x06, status 0x09, control 0x0A/0x0B,
 * continuous mode example uses 0x0A = 0x63 and soft reset 0x0B = 0x80 then 0x00.
 * I2C write/read sequences in the datasheet show the 7-bit slave address as 0x7C.
 */
#define QMC_ADDR_7BIT   0x7Cu
#define QMC_REG_CHIP_ID 0x00
#define QMC_REG_DATA_X0 0x01
#define QMC_REG_STATUS   0x09
#define QMC_REG_CTRL1    0x0A
#define QMC_REG_CTRL2    0x0B
#define QMC_CHIP_ID_VAL  0x90

#define QMC_CTRL1_CONT   0x63
#define QMC_CTRL2_RESET  0x80
#define QMC_CTRL2_CLEAR  0x00

/* Keep raw counts here; calibrate in your fusion layer if needed */
#define QMC_MAG_SCALE    1.0f

static int qmc6309_wait_ready(void)
{
	for (int i = 0; i < 50; i++) {
		uint8_t st = 0;
		if (icm_i2cm_read_reg(QMC_ADDR_7BIT, QMC_REG_STATUS, &st) != 0) {
			return -EIO;
		}
		if (st & 0x01) {
			return 0;
		}
		k_busy_wait(2000);
	}
	return -ETIMEDOUT;
}

static bool qmc6309_init(void)
{
	uint8_t id = 0;

	if (icm_i2cm_read_reg(QMC_ADDR_7BIT, QMC_REG_CHIP_ID, &id) != 0) {
		printk("QMC6309: no ACK on 0x%02X\n", QMC_ADDR_7BIT);
		return false;
	}

	printk("QMC6309: CHIP_ID=0x%02X\n", id);

	if (id != QMC_CHIP_ID_VAL) {
		printk("QMC6309: unexpected chip id, continuing anyway\n");
	}

	if (icm_i2cm_write_reg(QMC_ADDR_7BIT, QMC_REG_CTRL2, QMC_CTRL2_RESET) != 0) {
		printk("QMC6309: soft reset failed\n");
		return false;
	}
	k_sleep(K_MSEC(10));

	if (icm_i2cm_write_reg(QMC_ADDR_7BIT, QMC_REG_CTRL2, QMC_CTRL2_CLEAR) != 0) {
		printk("QMC6309: clear reset failed\n");
		return false;
	}
	k_sleep(K_MSEC(2));

	if (icm_i2cm_write_reg(QMC_ADDR_7BIT, QMC_REG_CTRL1, QMC_CTRL1_CONT) != 0) {
		printk("QMC6309: continuous-mode write failed\n");
		return false;
	}
	k_sleep(K_MSEC(10));

	uint8_t st = 0;
	if (icm_i2cm_read_reg(QMC_ADDR_7BIT, QMC_REG_STATUS, &st) == 0) {
		printk("QMC6309: status=0x%02X\n", st);
	}

	printk("QMC6309: OK via ICM I2CM @0x%02X\n", QMC_ADDR_7BIT);
	return true;
}

static void qmc6309_read(ImuData *d)
{
	static float last_mx, last_my, last_mz;

	if (qmc6309_wait_ready() != 0) {
		d->mx = last_mx;
		d->my = last_my;
		d->mz = last_mz;
		return;
	}

	uint8_t buf[6];
	if (icm_i2cm_read_bytes(QMC_ADDR_7BIT, QMC_REG_DATA_X0, buf, 6) != 0) {
		d->mx = last_mx;
		d->my = last_my;
		d->mz = last_mz;
		return;
	}

	int16_t mx = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t my = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t mz = (int16_t)((buf[5] << 8) | buf[4]);

	d->mx = last_mx = mx * QMC_MAG_SCALE;
	d->my = last_my = my * QMC_MAG_SCALE;
	d->mz = last_mz = mz * QMC_MAG_SCALE;
}

/* ─── sensor thread ──────────────────────────────────────────────── */
static ImuData latest;
static bool data_ready;
static K_MUTEX_DEFINE(dtqsys_mutex);

static void dtqsys_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sleep(K_MSEC(1500)); /* wait for USB CDC */

	/* SPI2 for ICM-45686 */
	spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
	if (!device_is_ready(spi_dev)) {
		printk("DTQSYS: spi2 not ready\n");
		return;
	}

	cs_gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(cs_gpio_dev)) {
		printk("DTQSYS: gpio0 not ready\n");
		return;
	}

	gpio_pin_configure(cs_gpio_dev, ICM_CS_PIN, GPIO_OUTPUT_HIGH); /* CS deasserted */

	/*
	 * Power-on sequence for sensors: P0.5=HIGH, P0.20=LOW, P0.31=HIGH.
	 * Give time for rails to stabilise before any access.
	 */
	gpio_pin_configure(cs_gpio_dev, 5,  GPIO_OUTPUT_HIGH);
	gpio_pin_configure(cs_gpio_dev, 20, GPIO_OUTPUT_LOW);
	gpio_pin_configure(cs_gpio_dev, 31, GPIO_OUTPUT_HIGH);
	k_sleep(K_MSEC(100));

	printk("DTQSYS: starting sensor init\n");

	bool icm_ok = icm45686_init();
	bool qmc_ok = false;

	if (icm_ok) {
		qmc_ok = qmc6309_init();
	}

	printk("DTQSYS: init done icm=%d qmc=%d\n", icm_ok, qmc_ok);

	uint32_t tick = 0;
	while (1) {
		if (icm_ok) {
			ImuData d = { 0 };
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
			if (icm_ok && !qmc_ok) {
				qmc_ok = qmc6309_init();
			}
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