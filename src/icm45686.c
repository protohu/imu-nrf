#include "imu.h"
#include "imu_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

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

/* ─── ICM-45686 MREG indirect register access (IREG mechanism) ───── */
/*
 * I2CM registers live in MREG2 (address range 0xa200-0xa2FF).
 * Access requires the IREG burst protocol (TDK inv_imu_transport.c):
 *
 * Write: single SPI burst with CS held low for all 4 bytes:
 *   [0x7C(cmd), addr_h, addr_l, data]
 *   → auto-increments: 0x7C→addr_h, 0x7D→addr_l, 0x7E→data
 *
 * Read: 2-byte burst write to set address, then separate 1-byte reads:
 *   TX: [0x7C(cmd), addr_h, addr_l]   (one CS-low transaction)
 *   RX: icm_spi_read(0x7E, ...)        (separate transaction, addr auto-increments)
 */
#define ICM_IREG_ADDR_H  0x7C  /* IREG_ADDR_15_8 — direct SPI reg, write command byte */
#define ICM_IREG_DATA    0x7E  /* IREG_DATA       — direct SPI reg */

static void icm_mreg_write(uint16_t addr, uint8_t val)
{
	uint8_t tx[4] = {ICM_IREG_ADDR_H, addr >> 8, addr & 0xFF, val};
	const struct spi_buf txb = {.buf = tx, .len = 4};
	const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
	cs_low();
	k_busy_wait(4);
	spi_write(spi_dev, spi_cfg, &txs);
	cs_high();
	k_busy_wait(4);
}

static uint8_t icm_mreg_read(uint16_t addr)
{
	/* Step 1: set IREG address via 2-byte burst write */
	uint8_t tx[3] = {ICM_IREG_ADDR_H, addr >> 8, addr & 0xFF};
	const struct spi_buf txb = {.buf = tx, .len = 3};
	const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
	cs_low();
	k_busy_wait(4);
	spi_write(spi_dev, spi_cfg, &txs);
	cs_high();
	/* Step 2: read 1 byte from IREG_DATA (MREG pointer advances after each access) */
	k_busy_wait(4);
	uint8_t val = 0;
	icm_spi_read(ICM_IREG_DATA, &val, 1);
	return val;
}

/* Read N consecutive MREG bytes — MREG pointer auto-increments after each IREG_DATA read. */
static void icm_mreg_read_burst(uint16_t addr, uint8_t *buf, int n)
{
	uint8_t tx[3] = {ICM_IREG_ADDR_H, addr >> 8, addr & 0xFF};
	const struct spi_buf txb = {.buf = tx, .len = 3};
	const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
	cs_low();
	k_busy_wait(4);
	spi_write(spi_dev, spi_cfg, &txs);
	cs_high();
	k_busy_wait(4);
	for (int i = 0; i < n; i++) {
		icm_spi_read(ICM_IREG_DATA, &buf[i], 1);
		k_busy_wait(4);
	}
}

/* ─── ICM-45686 I2C master (I2CM) — controls QMC6309 on AUX1 ────── */
/*
 * I2CM MREG2 registers (all high byte = 0xa2):
 *   COMMAND_0      0xa206  — transaction descriptor (len, r/w, ch, endflag)
 *   DEV_PROFILE0   0xa20e  — reg addr to read from for slave 0
 *   DEV_PROFILE1   0xa20f  — 7-bit I2C addr of slave 0
 *   CONTROL        0xa216  — bit0=go, bit1=speed(0=400kHz,1=100kHz)
 *   STATUS         0xa218  — bit0=busy, bit1=done, bits[5:2]=errors
 *   EXT_DEV_STATUS 0xa21a  — bit0=1 if slave 0 NACKed
 *   RD_DATA0-5     0xa21b+ — bytes returned from slave
 *   WR_DATA0+      0xa233+ — bytes to send to slave (reg addr + payload)
 */
#define I2CM_COMMAND_0      0xa206u
#define I2CM_DEV_PROFILE0   0xa20eu
#define I2CM_DEV_PROFILE1   0xa20fu
#define I2CM_CONTROL        0xa216u
#define I2CM_STATUS         0xa218u
#define I2CM_EXT_DEV_STATUS 0xa21au
#define I2CM_RD_DATA0       0xa21bu
#define I2CM_WR_DATA0       0xa233u

/* Poll I2CM_STATUS until bit1 (done) is set; return status byte. */
static uint8_t icm_i2cm_wait(void)
{
	uint8_t st = 0;
	for (int i = 0; i < 200; i++) {
		st = icm_mreg_read(I2CM_STATUS);
		if (st & 0x02) {
			break;
		}
		k_busy_wait(10);
	}
	return st;
}

/*
 * Write one byte to a register of an I2C slave via ICM I2CM.
 * COMMAND_0 = 0x82: write, burstlen=2 (reg addr + 1 data byte), endflag.
 */
static int icm_i2cm_write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t val)
{
	icm_mreg_write(I2CM_DEV_PROFILE1, i2c_addr);
	icm_mreg_write(I2CM_WR_DATA0,     reg);
	icm_mreg_write(I2CM_WR_DATA0 + 1, val);
	icm_mreg_write(I2CM_COMMAND_0,    0x82); /* write, len=2(reg+data), endflag */
	icm_mreg_write(I2CM_CONTROL,      0x03); /* go + speed=100kHz */
	uint8_t st = icm_i2cm_wait();
	/* bits[5:2] are error flags; NACK is reported in EXT_DEV_STATUS */
	return (st & 0x3C) ? -EIO : 0;
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

	/*
	 * Enable AUX1 I2C master mode.
	 *
	 * Step 1: IOC_PAD_SCENARIO (0x2F, base register)
	 *   bit0 = aux1_enable=1, bits[2:1] = aux1_mode=0b01 (I2CM master) → write 0x03.
	 *   This is needed because the factory default may have aux1_enable=0.
	 *
	 * Step 2: IOC_PAD_SCENARIO_AUX_OVRD (0x30, override register)
	 *   Forces mode override regardless of base register:
	 *   aux1_enable_ovrd_val[0]=1, aux1_enable_ovrd[1]=1,
	 *   aux1_mode_ovrd_val[3:2]=0b01, aux1_mode_ovrd[4]=1 → 0x17.
	 */
	uint8_t ioc_pad = 0;
	icm_spi_read(0x2F, &ioc_pad, 1);
	printk("ICM-45686: IOC_PAD_SCENARIO before=0x%02X\n", ioc_pad);
	ioc_pad = (ioc_pad & ~0x07u) | 0x03u; /* aux1_enable=1, aux1_mode=I2CM */
	icm_spi_write(0x2F, ioc_pad);

	uint8_t aux_ovrd = 0;
	icm_spi_read(0x30, &aux_ovrd, 1);
	aux_ovrd = (aux_ovrd & ~0x1Fu) | 0x17u; /* enable ovrd + mode ovrd → I2CM */
	icm_spi_write(0x30, aux_ovrd);
	k_sleep(K_MSEC(5));

	icm_spi_read(0x2F, &ioc_pad, 1);
	icm_spi_read(0x30, &aux_ovrd, 1);
	printk("ICM-45686: IOC_PAD_SCENARIO=0x%02X AUX_OVRD=0x%02X (expect 0x03, 0x17)\n",
	       ioc_pad, aux_ovrd);

	/* Clock for I2CM is provided by the PLL already running for accel/gyro LN mode */

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

/* ─── QMC6309 via ICM-45686 I2C master (AUX1) ───────────────────── */
/*
 * QMC6309 sits on ICM AUX1 I2C pins (not on nRF I2C0).
 * All access goes through ICM's I2CM: nRF triggers a transaction via SPI,
 * ICM drives AUX1 SDA/SCL to communicate with QMC6309, result is read back.
 */
#define QMC_ADDR_7BIT   0x2Cu  /* QMC6309 fixed 7-bit I2C address */
#define QMC_REG_CHIP_ID 0x00
#define QMC_REG_DATA_X0 0x01   /* 6 bytes LE: X Y Z */
#define QMC_REG_CTRL1   0x0A
#define QMC_REG_CTRL2   0x0B
#define QMC_CHIP_ID_VAL 0x90
/* CTRL1: MODE=continuous(0b11), OSR=8x, LPF=off */
#define QMC_CTRL1_VAL   0x03
/* CTRL2: ODR=100Hz(0b011<<4), RNG=8G(0b10<<2), SET_RESET_ON */
#define QMC_CTRL2_VAL   0x38
/* ±8G: 1/4000 Gauss/LSB × 100 µT/Gauss */
#define QMC_MAG_SCALE   (100.0f / 4000.0f)

/*
 * Scan all 7-bit I2C addresses via ICM I2CM to find what is actually on AUX1.
 * Returns the found address (0 if nothing found).
 */
static uint8_t icm_i2cm_scan(void)
{
	printk("I2CM AUX1 scan 0x08-0x77:\n");
	int found = 0;
	uint8_t found_addr = 0;

	/* Pre-configure: read 1 byte from reg 0x00, endflag */
	icm_mreg_write(I2CM_DEV_PROFILE0, 0x00);
	icm_mreg_write(I2CM_COMMAND_0,    0x91);

	/* Print initial STATUS to detect stale done=1 (bit1) before first transaction */
	uint8_t init_st = icm_mreg_read(I2CM_STATUS);
	printk("  I2CM_STATUS before first trigger: 0x%02X\n", init_st);

	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		icm_mreg_write(I2CM_DEV_PROFILE1, addr);
		icm_mreg_write(I2CM_CONTROL, 0x03); /* go + speed=100kHz */
		k_busy_wait(150); /* 150µs: allow I2C addr phase to start (10 bits @ 100kHz) */
		uint8_t st = icm_i2cm_wait();
		uint8_t dev_st = icm_mreg_read(I2CM_EXT_DEV_STATUS);
		uint8_t val    = icm_mreg_read(I2CM_RD_DATA0);
		if (addr == 0x2C) {
			printk("  @0x2C: i2cm_st=0x%02X ext_dev=0x%02X rd=0x%02X\n",
			       st, dev_st, val);
		}
		if (!(dev_st & 0x01)) {  /* bit0=0 = ACK (device present) */
			printk("  found 0x%02X (reg0=0x%02X)\n", addr, val);
			if (!found_addr) {
				found_addr = addr;
			}
			found++;
		}
	}
	if (!found) {
		printk("  nothing found — check CSB pin (must be HIGH for I2C mode)\n");
	}
	return found_addr;
}

static bool qmc6309_init(void)
{
	/* Scan AUX1 I2C bus — finds real address or confirms no I2C device present */
	uint8_t scan_addr = icm_i2cm_scan();

	uint8_t use_addr = QMC_ADDR_7BIT;
	if (scan_addr && scan_addr != QMC_ADDR_7BIT) {
		printk("QMC6309: scan found 0x%02X, expected 0x%02X — using scan result\n",
		       scan_addr, QMC_ADDR_7BIT);
		use_addr = scan_addr;
	} else if (!scan_addr) {
		printk("QMC6309: no I2C device on AUX1 — SPI mode? CSB pulled low?\n");
		return false;
	}

	/* Verify chip ID */
	icm_mreg_write(I2CM_DEV_PROFILE0, QMC_REG_CHIP_ID);
	icm_mreg_write(I2CM_DEV_PROFILE1, use_addr);
	icm_mreg_write(I2CM_COMMAND_0,    0x91);
	icm_mreg_write(I2CM_CONTROL,      0x03); /* go + speed=100kHz */
	uint8_t st     = icm_i2cm_wait();
	uint8_t id     = icm_mreg_read(I2CM_RD_DATA0);
	uint8_t dev_st = icm_mreg_read(I2CM_EXT_DEV_STATUS);
	printk("QMC6309: CHIP_ID=0x%02X i2cm_status=0x%02X dev_status=0x%02X @0x%02X\n",
	       id, st, dev_st, use_addr);

	if (dev_st & 0x01) {
		printk("QMC6309: NACK — device disappeared after scan?\n");
		return false;
	}
	if (id != QMC_CHIP_ID_VAL) {
		printk("QMC6309: chip_id=0x%02X (expected 0x%02X) — continuing\n",
		       id, QMC_CHIP_ID_VAL);
	}

	/* Update working address for read loop */
	/* (soft reset + configure use use_addr inline below) */

	/* Soft reset, then configure */
	icm_i2cm_write_reg(use_addr, QMC_REG_CTRL2, 0x80);
	k_sleep(K_MSEC(10));
	icm_i2cm_write_reg(use_addr, QMC_REG_CTRL2, QMC_CTRL2_VAL);
	icm_i2cm_write_reg(use_addr, QMC_REG_CTRL1, QMC_CTRL1_VAL);
	k_sleep(K_MSEC(10));

	/* Pre-configure I2CM for the periodic 6-byte data reads.
	 * COMMAND_0 = 0x96: read, burstlen=6, ch_sel=0, endflag.
	 * These registers are not changed between reads, so set them once here. */
	icm_mreg_write(I2CM_DEV_PROFILE0, QMC_REG_DATA_X0);
	icm_mreg_write(I2CM_DEV_PROFILE1, use_addr);
	icm_mreg_write(I2CM_COMMAND_0,    0x96);

	printk("QMC6309: OK via ICM I2CM @0x%02X\n", use_addr);
	return true;
}

static void qmc6309_read(ImuData *d)
{
	static float last_mx, last_my, last_mz;

	/* Profile and command are pre-configured in qmc6309_init(); just trigger. */
	icm_mreg_write(I2CM_CONTROL, 0x03); /* go + speed=100kHz */
	uint8_t st = icm_i2cm_wait();
	if (st & 0x3C) { /* error bits [5:2] */
		d->mx = last_mx;
		d->my = last_my;
		d->mz = last_mz;
		return;
	}

	uint8_t buf[6];
	icm_mreg_read_burst(I2CM_RD_DATA0, buf, 6);

	int16_t mx = (int16_t)((buf[1] << 8) | buf[0]);
	int16_t my = (int16_t)((buf[3] << 8) | buf[2]);
	int16_t mz = (int16_t)((buf[5] << 8) | buf[4]);

	d->mx = last_mx = mx * QMC_MAG_SCALE;
	d->my = last_my = my * QMC_MAG_SCALE;
	d->mz = last_mz = mz * QMC_MAG_SCALE;
}

/* ─── QMC6309 SPI probe (direct nRF SPI2, different CS pins) ────── */
/*
 * QMC6309 SPI uses Mode 3 (CPOL=1 CPHA=1). Read reg 0x00 (CHIP_ID, expect 0x90).
 * Tries all exposed P0 and P1 pins as CS candidates.
 * SPI2: MOSI=P1.8, MISO=P1.9 (pull-up), SCK=P0.7.
 */
static void qmc6309_spi_probe(void)
{
	static const struct spi_config qmc_spi_m3 = {
		.frequency = 500000,
		.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA,
		.slave = 0,
	};

	/* P0 candidates (skip ICM CS=P0.4 and SCK=P0.7) */
	static const uint8_t p0_pins[] = {2, 5, 6, 8, 9, 10, 11, 17, 20, 29, 31};
	static const char *p0_names[] = {
		"P0.2","P0.5","P0.6","P0.8","P0.9","P0.10","P0.11","P0.17","P0.20","P0.29","P0.31"
	};
	/* P1 candidates (skip MOSI=P1.8, MISO=P1.9) */
	static const uint8_t p1_pins[] = {0, 4, 6, 11, 13, 15};
	static const char *p1_names[] = {"P1.0","P1.4","P1.6","P1.11","P1.13","P1.15"};

	const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	printk("QMC6309 SPI probe Mode3 (MOSI=P1.8 MISO=P1.9 SCK=P0.7):\n");

	/* Try P0 pins */
	for (int i = 0; i < ARRAY_SIZE(p0_pins); i++) {
		uint8_t pin = p0_pins[i];
		gpio_pin_configure(cs_gpio_dev, pin, GPIO_OUTPUT_HIGH);
		k_busy_wait(100);

		uint8_t tx[2] = {0x80, 0x00};
		uint8_t rx[2] = {0, 0};
		const struct spi_buf txb = {.buf = tx, .len = 2};
		const struct spi_buf rxb = {.buf = rx, .len = 2};
		const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
		const struct spi_buf_set rxs = {.buffers = &rxb, .count = 1};

		gpio_pin_set_raw(cs_gpio_dev, pin, 0);
		k_busy_wait(5);
		spi_transceive(spi_dev, &qmc_spi_m3, &txs, &rxs);
		gpio_pin_set_raw(cs_gpio_dev, pin, 1);

		bool hit = (rx[1] == 0x90 || rx[0] == 0x90);
		if (hit || rx[1] != 0xFF || rx[0] != 0xFF) {
			printk("  CS=%s: rx[0]=0x%02X rx[1]=0x%02X%s\n",
			       p0_names[i], rx[0], rx[1], hit ? " ← QMC FOUND!" : " ← non-FF");
		}
	}

	/* Try P1 pins */
	if (device_is_ready(gpio1)) {
		for (int i = 0; i < ARRAY_SIZE(p1_pins); i++) {
			uint8_t pin = p1_pins[i];
			gpio_pin_configure(gpio1, pin, GPIO_OUTPUT_HIGH);
			k_busy_wait(100);

			uint8_t tx[2] = {0x80, 0x00};
			uint8_t rx[2] = {0, 0};
			const struct spi_buf txb = {.buf = tx, .len = 2};
			const struct spi_buf rxb = {.buf = rx, .len = 2};
			const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
			const struct spi_buf_set rxs = {.buffers = &rxb, .count = 1};

			gpio_pin_set_raw(gpio1, pin, 0);
			k_busy_wait(5);
			spi_transceive(spi_dev, &qmc_spi_m3, &txs, &rxs);
			gpio_pin_set_raw(gpio1, pin, 1);

			bool hit = (rx[1] == 0x90 || rx[0] == 0x90);
			if (hit || rx[1] != 0xFF || rx[0] != 0xFF) {
				printk("  CS=%s: rx[0]=0x%02X rx[1]=0x%02X%s\n",
				       p1_names[i], rx[0], rx[1], hit ? " ← QMC FOUND!" : " ← non-FF");
			}
		}
	}
	printk("QMC SPI probe done (only non-FF results printed above)\n");
}

/*
 * CSB GPIO sweep: set each GPIO HIGH one at a time, probe I2CM at QMC_ADDR_7BIT.
 * If QMC ACKs after setting pin X, that pin controls CSB.
 */
static void find_qmc_csb_gpio(void)
{
	const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	/* Pre-configure I2CM: read 1 byte from reg 0x00 of QMC */
	icm_mreg_write(I2CM_DEV_PROFILE0, 0x00);
	icm_mreg_write(I2CM_COMMAND_0,    0x91);
	icm_mreg_write(I2CM_DEV_PROFILE1, QMC_ADDR_7BIT);

	printk("CSB sweep: toggling each GPIO HIGH, probing I2CM @0x%02X:\n", QMC_ADDR_7BIT);

	bool found = false;

	/* P0: skip ICM_CS=P0.4, SCK=P0.7, I2C0_SCL=P0.22, I2C0_SDA=P0.24 */
	for (uint8_t pin = 0; pin <= 31; pin++) {
		if (pin == ICM_CS_PIN || pin == 7 || pin == 22 || pin == 24) {
			continue;
		}
		gpio_pin_configure(cs_gpio_dev, pin, GPIO_OUTPUT_HIGH);
		k_busy_wait(2000); /* 2ms: QMC needs time to switch from SPI→I2C */

		icm_mreg_write(I2CM_CONTROL, 0x03);
		icm_i2cm_wait();
		uint8_t dev_st = icm_mreg_read(I2CM_EXT_DEV_STATUS);

		if (!(dev_st & 0x01)) { /* ACK! */
			uint8_t id = icm_mreg_read(I2CM_RD_DATA0);
			printk("  QMC FOUND: CSB = P0.%d HIGH  chip_id=0x%02X\n", pin, id);
			found = true;
		}
		gpio_pin_configure(cs_gpio_dev, pin, GPIO_INPUT);
		k_busy_wait(200);
	}

	/* P1: skip MOSI=P1.8, MISO=P1.9 */
	if (device_is_ready(gpio1)) {
		for (uint8_t pin = 0; pin <= 15; pin++) {
			if (pin == 8 || pin == 9) {
				continue;
			}
			gpio_pin_configure(gpio1, pin, GPIO_OUTPUT_HIGH);
			k_busy_wait(2000);

			icm_mreg_write(I2CM_CONTROL, 0x03);
			icm_i2cm_wait();
			uint8_t dev_st = icm_mreg_read(I2CM_EXT_DEV_STATUS);

			if (!(dev_st & 0x01)) { /* ACK! */
				uint8_t id = icm_mreg_read(I2CM_RD_DATA0);
				printk("  QMC FOUND: CSB = P1.%d HIGH  chip_id=0x%02X\n", pin, id);
				found = true;
			}
			gpio_pin_configure(gpio1, pin, GPIO_INPUT);
			k_busy_wait(200);
		}
	}

	if (!found) {
		printk("  no GPIO found — CSB нужна аппаратная подтяжка к VCC\n");
	}
}

/* ─── nRF I2C0 diagnostic scan (P0.24=SDA P0.22=SCL) ────────────── */
static void nrf_i2c0_scan(void)
{
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!device_is_ready(i2c)) {
		printk("nRF I2C0: not ready\n");
		return;
	}
	printk("nRF I2C0 scan (P0.24 SDA / P0.22 SCL) 0x08-0x77:\n");
	int found = 0;
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t reg = 0x00, val = 0;
		if (i2c_write_read(i2c, addr, &reg, 1, &val, 1) == 0) {
			printk("  found 0x%02X (reg0=0x%02X)\n", addr, val);
			found++;
		}
	}
	if (!found) {
		printk("  nothing found\n");
	}
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

	/*
	 * Power-on sequence for sensors: P0.5=HIGH (open-source VCC), P0.20=LOW (GND path),
	 * P0.31=HIGH (VCC rail). Give 100ms for rails to stabilise before any I2C/SPI access.
	 */
	gpio_pin_configure(cs_gpio_dev, 5,  GPIO_OUTPUT_HIGH);
	gpio_pin_configure(cs_gpio_dev, 20, GPIO_OUTPUT_LOW);
	gpio_pin_configure(cs_gpio_dev, 31, GPIO_OUTPUT_HIGH);
	k_sleep(K_MSEC(100));

	printk("DTQSYS: starting sensor init\n");

	bool icm_ok = icm45686_init();

	/* Sweep all GPIOs to find which one controls QMC6309 CSB */
	if (icm_ok) {
		find_qmc_csb_gpio();
	}

	bool qmc_ok = icm_ok ? qmc6309_init() : false;

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
