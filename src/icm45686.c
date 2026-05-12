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
#include <stdlib.h>
#include <math.h>

/* ==================== ICM-45686 ==================== */
#define ICM_REG_ACCEL_X0     0x00
#define ICM_REG_PWR_MGMT0    0x10
#define ICM_REG_ACCEL_CFG0   0x1B
#define ICM_REG_GYRO_CFG0    0x1C
#define ICM_REG_WHO_AM_I     0x72

#define ICM_SPI_READ         0x80
#define ICM_PWR_LN           0x0F
#define ICM_ACCEL_CFG_VAL    (0x40 | 0x09)
#define ICM_GYRO_CFG_VAL     (0x00 | 0x09)

#define ICM_ACCEL_SCALE      (2.0f * 9.81f / 32768.0f)
#define ICM_GYRO_SCALE       (2000.0f * 3.14159265f / (180.0f * 32768.0f))

#define ICM_CS_PIN 4

static const struct device *cs_gpio_dev;
static const struct device *spi_dev;
static K_MUTEX_DEFINE(icm_bus_mutex);

/* Частота SPI 250 кГц – стабильно */
static const struct spi_config spi_cfg = {
    .frequency = 250000,
    .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
};

static inline void cs_low(void)  { gpio_pin_set_raw(cs_gpio_dev, ICM_CS_PIN, 0); }
static inline void cs_high(void) { gpio_pin_set_raw(cs_gpio_dev, ICM_CS_PIN, 1); }

/* ==================== SPI low-level ==================== */
static int icm_spi_read_nolock(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len > 12) return -EINVAL;
    uint8_t tx[13] = { ICM_SPI_READ | reg };
    uint8_t rx[13] = { 0 };

    const struct spi_buf tx_bufs[] = {{ .buf = tx, .len = len + 1 }};
    const struct spi_buf rx_bufs[] = {{ .buf = rx, .len = len + 1 }};
    const struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 1 };
    const struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 1 };

    cs_low();
    k_busy_wait(15);
    int rc = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    cs_high();
    k_busy_wait(10);

    if (rc == 0) memcpy(buf, rx + 1, len);
    return rc;
}

static int icm_spi_write_nolock(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & ~ICM_SPI_READ, val };
    const struct spi_buf tx_bufs[] = {{ .buf = tx, .len = 2 }};
    const struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = 1 };

    cs_low();
    k_busy_wait(15);
    int rc = spi_write(spi_dev, &spi_cfg, &tx_set);
    cs_high();
    k_busy_wait(10);
    return rc;
}

static int icm_spi_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    k_mutex_lock(&icm_bus_mutex, K_FOREVER);
    int rc = icm_spi_read_nolock(reg, buf, len);
    k_mutex_unlock(&icm_bus_mutex);
    return rc;
}

static int icm_spi_write(uint8_t reg, uint8_t val) {
    k_mutex_lock(&icm_bus_mutex, K_FOREVER);
    int rc = icm_spi_write_nolock(reg, val);
    k_mutex_unlock(&icm_bus_mutex);
    return rc;
}

/* ==================== I2CM access via MREG ==================== */
#define ICM_IREG_ADDR_H   0x7C
#define ICM_IREG_DATA     0x7E

static int icm_mreg_write_nolock(uint16_t addr, uint8_t val)
{
    uint8_t tx[4] = { ICM_IREG_ADDR_H, addr >> 8, addr & 0xFF, val };
    const struct spi_buf txb = { .buf = tx, .len = 4 };
    const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
    cs_low();
    k_busy_wait(15);
    int rc = spi_write(spi_dev, &spi_cfg, &txs);
    cs_high();
    k_busy_wait(10);
    return rc;
}

static int icm_mreg_read_nolock(uint16_t addr, uint8_t *val)
{
    uint8_t tx[3] = { ICM_IREG_ADDR_H, addr >> 8, addr & 0xFF };
    const struct spi_buf txb = { .buf = tx, .len = 3 };
    const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
    cs_low();
    k_busy_wait(15);
    int rc = spi_write(spi_dev, &spi_cfg, &txs);
    cs_high();
    k_busy_wait(10);
    if (rc) return rc;
    return icm_spi_read_nolock(ICM_IREG_DATA, val, 1);
}

/* ==================== I2CM master registers ==================== */
#define I2CM_COMMAND_0       0xa206u
#define I2CM_DEV_PROFILE0    0xa20eu
#define I2CM_DEV_PROFILE1    0xa20fu
#define I2CM_CONTROL         0xa216u
#define I2CM_STATUS          0xa218u
#define I2CM_EXT_DEV_STATUS  0xa21au
#define I2CM_RD_DATA0        0xa21bu
#define I2CM_WR_DATA0        0xa233u

#define I2CM_CTRL_WRITE      (0x08u | 0x01u)   // standard mode + GO
#define I2CM_CTRL_READ       (0x40u | 0x08u | 0x01u) // restart + standard + GO

static void icm_i2cm_reset_nolock(void)
{
    uint8_t dummy;
    icm_mreg_read_nolock(I2CM_STATUS, &dummy);
    icm_mreg_read_nolock(I2CM_EXT_DEV_STATUS, &dummy);
}

/* Неблокирующее ожидание: не более 2 мс */
static int icm_i2cm_wait_nolock(void)
{
    uint8_t st = 0;
    for (int i = 0; i < 18; i++) {   // 18 * 100 мкс = 1.8 мс
        if (icm_mreg_read_nolock(I2CM_STATUS, &st) != 0) return -EIO;
        if ((st & 0x02) && !(st & 0x01)) {
            return (st & 0x3C) ? -EIO : 0;
        }
        k_busy_wait(100);
    }
    return -ETIMEDOUT;  // слишком долго – отказываемся от этого кадра
}

static int icm_i2cm_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    k_mutex_lock(&icm_bus_mutex, K_FOREVER);
    icm_i2cm_reset_nolock();
    int rc = 0;
    rc |= icm_mreg_write_nolock(I2CM_DEV_PROFILE0, reg);
    rc |= icm_mreg_write_nolock(I2CM_DEV_PROFILE1, addr);
    rc |= icm_mreg_write_nolock(I2CM_COMMAND_0, 0x91);
    rc |= icm_mreg_write_nolock(I2CM_CONTROL, I2CM_CTRL_READ);
    if (rc == 0 && icm_i2cm_wait_nolock() == 0) {
        uint8_t dev_st;
        icm_mreg_read_nolock(I2CM_EXT_DEV_STATUS, &dev_st);
        if (!(dev_st & 0x01))
            rc = icm_mreg_read_nolock(I2CM_RD_DATA0, val);
        else
            rc = -EIO;
    }
    k_mutex_unlock(&icm_bus_mutex);
    return rc;
}

static int icm_i2cm_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len == 0 || len > 15) return -EINVAL;
    k_mutex_lock(&icm_bus_mutex, K_FOREVER);
    icm_i2cm_reset_nolock();
    int rc = 0;
    rc |= icm_mreg_write_nolock(I2CM_DEV_PROFILE0, reg);
    rc |= icm_mreg_write_nolock(I2CM_DEV_PROFILE1, addr);
    rc |= icm_mreg_write_nolock(I2CM_COMMAND_0, 0x90 | (len & 0x0F));
    rc |= icm_mreg_write_nolock(I2CM_CONTROL, I2CM_CTRL_READ);
    if (rc == 0 && icm_i2cm_wait_nolock() == 0) {
        uint8_t dev_st;
        icm_mreg_read_nolock(I2CM_EXT_DEV_STATUS, &dev_st);
        if (!(dev_st & 0x01)) {
            for (int i = 0; i < len; i++) {
                if (icm_mreg_read_nolock(I2CM_RD_DATA0 + i, &buf[i]) != 0) {
                    rc = -EIO;
                    break;
                }
            }
        } else rc = -EIO;
    }
    k_mutex_unlock(&icm_bus_mutex);
    return rc;
}

static int icm_i2cm_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    k_mutex_lock(&icm_bus_mutex, K_FOREVER);
    icm_i2cm_reset_nolock();
    int rc = 0;
    rc |= icm_mreg_write_nolock(I2CM_DEV_PROFILE1, addr);
    rc |= icm_mreg_write_nolock(I2CM_WR_DATA0, reg);
    rc |= icm_mreg_write_nolock(I2CM_WR_DATA0 + 1, val);
    rc |= icm_mreg_write_nolock(I2CM_COMMAND_0, 0x82);
    rc |= icm_mreg_write_nolock(I2CM_CONTROL, I2CM_CTRL_WRITE);
    if (rc == 0) icm_i2cm_wait_nolock(); // не проверяем результат – пишем один раз
    k_mutex_unlock(&icm_bus_mutex);
    return rc;
}

/* ==================== ICM read with validation ==================== */
static bool icm45686_read_valid(ImuData *d)
{
    static int bad_frames = 0;
    static uint32_t last_reset_ms = 0;

    for (int attempt = 0; attempt < 3; attempt++) {
        uint8_t buf[12];
        if (icm_spi_read(ICM_REG_ACCEL_X0, buf, 12) != 0) continue;

        int16_t ax = (int16_t)((buf[1]<<8)|buf[0]);
        int16_t ay = (int16_t)((buf[3]<<8)|buf[2]);
        int16_t az = (int16_t)((buf[5]<<8)|buf[4]);
        int16_t gx = (int16_t)((buf[7]<<8)|buf[6]);
        int16_t gy = (int16_t)((buf[9]<<8)|buf[8]);
        int16_t gz = (int16_t)((buf[11]<<8)|buf[10]);

        /* Норма вектора ≥ 0.3g (5000 raw): не зависит от ориентации датчика */
        int64_t anorm2 = (int64_t)ax*ax + (int64_t)ay*ay + (int64_t)az*az;
        if (anorm2 < 25000000LL) {
            bad_frames++;
            if (bad_frames >= 4) {
                uint32_t now = k_uptime_get_32();
                if (now - last_reset_ms > 1500) {
                    printk("ICM bad frames, soft reset\n");
                    icm_spi_write(0x01, 0x01);
                    k_sleep(K_MSEC(12));
                    last_reset_ms = now;
                }
                bad_frames = 0;
            }
            continue;
        }
        /* Гироскоп ±2000 dps → raw ±32767, 2000 dps = 16384 LSB */
        if (abs(gx) > 29000 || abs(gy) > 29000 || abs(gz) > 29000) {
            bad_frames++;
            continue;
        }

        bad_frames = 0;
        d->ax = ax * ICM_ACCEL_SCALE;
        d->ay = ay * ICM_ACCEL_SCALE;
        d->az = az * ICM_ACCEL_SCALE;
        d->gx = gx * ICM_GYRO_SCALE;
        d->gy = gy * ICM_GYRO_SCALE;
        d->gz = gz * ICM_GYRO_SCALE;
        d->timestamp_us = (uint32_t)(k_uptime_get() * 1000U);
        return true;
    }
    return false;
}

/* ==================== ICM-45686 initialisation ==================== */
static bool icm45686_init(void)
{
    cs_high();
    k_sleep(K_MSEC(10));

    icm_spi_write(0x01, 0x01);
    k_sleep(K_MSEC(12));

    uint8_t who = 0;
    icm_spi_read(ICM_REG_WHO_AM_I, &who, 1);
    printk("ICM WHO_AM_I = 0x%02X\n", who);

    icm_spi_write(ICM_REG_PWR_MGMT0, 0x00);
    k_sleep(K_MSEC(2));
    icm_spi_write(ICM_REG_ACCEL_CFG0, ICM_ACCEL_CFG_VAL);
    icm_spi_write(ICM_REG_GYRO_CFG0, ICM_GYRO_CFG_VAL);
    icm_spi_write(ICM_REG_PWR_MGMT0, ICM_PWR_LN);
    k_sleep(K_MSEC(50));

    uint8_t tmp;
    icm_spi_read(0x2F, &tmp, 1);
    tmp = (tmp & ~0x07u) | 0x03u;
    icm_spi_write(0x2F, tmp);
    icm_spi_read(0x30, &tmp, 1);
    tmp = (tmp & ~0x1Fu) | 0x17u;
    icm_spi_write(0x30, tmp);
    k_sleep(K_MSEC(5));

    return true;
}

/* ==================== QMC6309 via I2CM ==================== */
#define QMC_ADDR_7BIT     0x7Cu
#define QMC_REG_CHIP_ID   0x00
#define QMC_REG_DATA_X0   0x01
#define QMC_REG_STATUS    0x09
#define QMC_REG_CTRL1     0x0A
#define QMC_REG_CTRL2     0x0B
#define QMC_CHIP_ID_VAL   0x90
#define QMC_CTRL1_CONT    0x63
#define QMC_CTRL2_RESET   0x80
#define QMC_CTRL2_CLEAR   0x00

#define QMC_MAG_SCALE        0.1f
#define QMC_READ_INTERVAL_MS 30u   // редко, чтобы не напрягать шину

static bool qmc_has_last = false;
static uint32_t qmc_next_poll_ms = 0;
static int16_t qmc_last_raw[3] = {0};
static float qmc_last[3] = {0};

static bool qmc6309_init(void)
{
    uint8_t id = 0;
    if (icm_i2cm_read_reg(QMC_ADDR_7BIT, QMC_REG_CHIP_ID, &id) != 0) {
        printk("QMC6309: no ACK\n");
        return false;
    }
    printk("QMC6309 CHIP_ID = 0x%02X\n", id);

    icm_i2cm_write_reg(QMC_ADDR_7BIT, QMC_REG_CTRL2, QMC_CTRL2_RESET);
    k_sleep(K_MSEC(10));
    icm_i2cm_write_reg(QMC_ADDR_7BIT, QMC_REG_CTRL2, QMC_CTRL2_CLEAR);
    k_sleep(K_MSEC(5));
    icm_i2cm_write_reg(QMC_ADDR_7BIT, QMC_REG_CTRL1, QMC_CTRL1_CONT);
    k_sleep(K_MSEC(10));

    printk("QMC6309 ready\n");
    return true;
}

static void qmc6309_update(ImuData *d)
{
    uint32_t now = k_uptime_get_32();

    /* Если ещё не наступило время следующего опроса – возвращаем последние данные */
    if (qmc_has_last && (int32_t)(now - qmc_next_poll_ms) < 0) {
        d->mx = qmc_last[0]; d->my = qmc_last[1]; d->mz = qmc_last[2];
        return;
    }
    qmc_next_poll_ms = now + QMC_READ_INTERVAL_MS;

    /* Пытаемся прочитать. Если что-то идёт не так – просто выходим с last */
    uint8_t st = 0;
    if (icm_i2cm_read_reg(QMC_ADDR_7BIT, QMC_REG_STATUS, &st) != 0 ||
        (st & 0x01) == 0 || (st & 0x02) != 0) {
        d->mx = qmc_last[0]; d->my = qmc_last[1]; d->mz = qmc_last[2];
        return;
    }

    uint8_t buf[6];
    if (icm_i2cm_read_bytes(QMC_ADDR_7BIT, QMC_REG_DATA_X0, buf, 6) != 0) {
        d->mx = qmc_last[0]; d->my = qmc_last[1]; d->mz = qmc_last[2];
        return;
    }

    int16_t raw[3] = {
        (int16_t)((buf[1]<<8)|buf[0]),
        (int16_t)((buf[3]<<8)|buf[2]),
        (int16_t)((buf[5]<<8)|buf[4])
    };

    /* Физическая проверка: норма поля должна быть 20–100 µT → raw 200–1000 */
    float norm = sqrtf((float)(raw[0]*raw[0] + raw[1]*raw[1] + raw[2]*raw[2]));
    if (norm < 140.0f || norm > 1300.0f) {
        d->mx = qmc_last[0]; d->my = qmc_last[1]; d->mz = qmc_last[2];
        return;
    }

    /* Отбрасываем резкие скачки более 4000 LSB */
    if (qmc_has_last) {
        if (abs(raw[0] - qmc_last_raw[0]) > 4000 ||
            abs(raw[1] - qmc_last_raw[1]) > 4000 ||
            abs(raw[2] - qmc_last_raw[2]) > 4000) {
            d->mx = qmc_last[0]; d->my = qmc_last[1]; d->mz = qmc_last[2];
            return;
        }
    }

    qmc_last_raw[0] = raw[0]; qmc_last_raw[1] = raw[1]; qmc_last_raw[2] = raw[2];
    qmc_has_last = true;

    /* Плавный фильтр (alpha = 0.18) */
    const float alpha = 0.18f;
    float nv[3] = { raw[0] * QMC_MAG_SCALE, raw[1] * QMC_MAG_SCALE, raw[2] * QMC_MAG_SCALE };
    if (qmc_last[0] == 0.0f && qmc_last[1] == 0.0f && qmc_last[2] == 0.0f) {
        qmc_last[0] = nv[0]; qmc_last[1] = nv[1]; qmc_last[2] = nv[2];
    } else {
        for (int i = 0; i < 3; i++)
            qmc_last[i] += alpha * (nv[i] - qmc_last[i]);
    }

    d->mx = qmc_last[0]; d->my = qmc_last[1]; d->mz = qmc_last[2];
}

/* ==================== Sensor thread ==================== */
static ImuData latest = {0};
static bool data_ready = false;
static K_MUTEX_DEFINE(dtqsys_mutex);

static void dtqsys_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    k_sleep(K_MSEC(1500));

    spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
    cs_gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

    gpio_pin_configure(cs_gpio_dev, ICM_CS_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(cs_gpio_dev, 5,  GPIO_OUTPUT_HIGH);
    gpio_pin_configure(cs_gpio_dev, 20, GPIO_OUTPUT_LOW);
    gpio_pin_configure(cs_gpio_dev, 31, GPIO_OUTPUT_HIGH);
    k_sleep(K_MSEC(100));

    bool icm_ok = icm45686_init();
    bool qmc_ok = icm_ok ? qmc6309_init() : false;
    printk("DTQSYS: ICM=%d QMC=%d\n", icm_ok, qmc_ok);

    int64_t next_tick = k_uptime_get() + 10;

    while (1) {
        if (icm_ok) {
            ImuData d = {0};
            if (icm45686_read_valid(&d)) {
                if (qmc_ok) qmc6309_update(&d);
                k_mutex_lock(&dtqsys_mutex, K_FOREVER);
                latest = d;
                data_ready = true;
                k_mutex_unlock(&dtqsys_mutex);
            }
        }

        int64_t delay = next_tick - k_uptime_get();
        if (delay > 0) k_sleep(K_MSEC((int32_t)delay));
        next_tick += 10;
    }
}

K_THREAD_DEFINE(dtqsys_tid, 2048, dtqsys_thread, NULL, NULL, NULL, 5, 0, 0);

/* ==================== IMU driver registration ==================== */
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
    .init = NULL,
    .get_data = dtqsys_get_data,
};

static int dtqsys_register(void)
{
    imu_register_driver(&dtqsys_driver);
    return 0;
}

SYS_INIT(dtqsys_register, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);