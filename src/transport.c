#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/shell/shell.h>
#include <hal/nrf_power.h>
#include "transport.h"

#define DFU_MAGIC_UF2 0x57

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static volatile int usb_state;

static void blink(int n, int ms)
{
	for (int i = 0; i < n; i++) {
		gpio_pin_set_dt(&led, 1);
		k_sleep(K_MSEC(ms));
		gpio_pin_set_dt(&led, 0);
		k_sleep(K_MSEC(ms));
	}
	k_sleep(K_MSEC(500));
}

static void usb_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_state = (int)status;
}

static int cmd_dfu(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Entering UF2 bootloader...");
	k_sleep(K_MSEC(100));
	nrf_power_gpregret_set(NRF_POWER, 0, DFU_MAGIC_UF2);
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

SHELL_CMD_REGISTER(dfu, NULL, "Reboot into UF2 bootloader", cmd_dfu);

int transport_init(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	blink(1, 300);

	int ret = usb_enable(usb_cb);
	if (ret != 0) {
		blink(10, 100);
		return ret;
	}
	blink(2, 300);

	for (int i = 0; i < 100; i++) {
		if (usb_state == USB_DC_CONFIGURED) break;
		k_sleep(K_MSEC(100));
	}
	blink((usb_state == USB_DC_CONFIGURED) ? 3 : 5, 300);

	return 0;
}
