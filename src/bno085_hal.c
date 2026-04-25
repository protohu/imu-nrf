#include "bno085_hal.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>

#define BNO085_ADDR 0x4B

static const struct device *i2c_dev;

static int bno_open(sh2_Hal_t *self)
{
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        printk("BNO085: i2c0 not ready\n");
        return -1;
    }

    k_sleep(K_MSEC(300));
    return 0;
}

static void bno_close(sh2_Hal_t *self)
{
    i2c_dev = NULL;
}

static int bno_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    /* First read: 4-byte header to get packet length */
    uint8_t hdr[4];
    if (i2c_read(i2c_dev, hdr, sizeof(hdr), BNO085_ADDR) != 0) {
        return 0;
    }

    uint16_t pkt_len = ((uint16_t)hdr[1] << 8) | hdr[0];
    pkt_len &= 0x7FFF; /* strip continuation bit */

    if (pkt_len < 4 || pkt_len > len) {
        return 0;
    }

    /* Second read: BNO085 resets its pointer at each I2C START condition,
     * so we must read the full packet (header + payload) from the beginning */
    if (i2c_read(i2c_dev, pBuffer, pkt_len, BNO085_ADDR) != 0) {
        return 0;
    }

    *t_us = (uint32_t)(k_uptime_get() * 1000U);
    return pkt_len;
}

static int bno_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    int ret = i2c_write(i2c_dev, pBuffer, len, BNO085_ADDR);
    if (ret != 0) {
        return -1; /* SHTP: -1 = abort, 0 = retry, >0 = success */
    }
    return len;
}

static uint32_t bno_get_time_us(sh2_Hal_t *self)
{
    return (uint32_t)(k_uptime_get() * 1000U);
}

sh2_Hal_t bno085_hal = {
    .open      = bno_open,
    .close     = bno_close,
    .read      = bno_read,
    .write     = bno_write,
    .getTimeUs = bno_get_time_us,
};
