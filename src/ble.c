#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include "ble.h"

bool notify_enabled;
struct bt_conn *current_conn;

static const struct bt_data *adv_ad;
static size_t adv_ad_len;
static const struct bt_data *adv_sd;
static size_t adv_sd_len;

static struct k_work_delayable adv_work;
static K_SEM_DEFINE(bt_ready_sem, 0, 1);

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			     struct bt_gatt_exchange_params *params)
{
	printk("MTU exchange %s, MTU=%u\n",
	       err == 0 ? "OK" : "FAIL", bt_gatt_get_mtu(conn));
}
static struct bt_gatt_exchange_params mtu_params = { .func = mtu_exchange_cb };

static void adv_restart(void)
{
	bt_le_adv_stop();
	int ret = bt_le_adv_start(BT_LE_ADV_CONN, adv_ad, adv_ad_len, adv_sd, adv_sd_len);
	if (ret && ret != -EALREADY) {
		printk("BLE adv restart failed: %d, retrying...\n", ret);
		k_work_schedule(&adv_work, K_MSEC(500));
	} else {
		printk("BLE advertising restarted\n");
	}
}

static void adv_work_handler(struct k_work *work)
{
	adv_restart();
}

static void adv_watchdog_thread(void *p1, void *p2, void *p3)
{
	k_sem_take(&bt_ready_sem, K_FOREVER);
	while (1) {
		k_sleep(K_SECONDS(3));
		if (current_conn != NULL) {
			continue;
		}
		printk("watchdog: forcing adv restart\n");
		adv_restart();
	}
}

K_THREAD_DEFINE(watchdog_tid, 1024, adv_watchdog_thread, NULL, NULL, NULL, 7, 0, 0);

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("BLE connect failed: %d\n", err);
		return;
	}
	current_conn = bt_conn_ref(conn);
	printk("BLE connected, MTU=%u\n", bt_gatt_get_mtu(conn));

	bt_gatt_exchange_mtu(conn, &mtu_params);

	/* 10–20 ms connection interval, 1 s supervision timeout */
	static const struct bt_le_conn_param param = {
		.interval_min = 8,   /* 8 × 1.25 ms = 10 ms */
		.interval_max = 16,  /* 16 × 1.25 ms = 20 ms */
		.latency = 0,
		.timeout = 100,      /* 100 × 10 ms = 1 s */
	};
	bt_conn_le_param_update(conn, &param);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("BLE disconnected: %d\n", reason);
	notify_enabled = false;
	bt_conn_unref(current_conn);
	current_conn = NULL;
	k_work_schedule(&adv_work, K_MSEC(100));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

int ble_init(const struct bt_data *ad, size_t ad_len,
             const struct bt_data *sd, size_t sd_len)
{
	adv_ad     = ad;
	adv_ad_len = ad_len;
	adv_sd     = sd;
	adv_sd_len = sd_len;

	k_work_init_delayable(&adv_work, adv_work_handler);

	int ret = bt_enable(NULL);
	if (ret != 0) {
		printk("BLE init failed: %d\n", ret);
		return ret;
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ad_len, sd, sd_len);
	if (ret != 0) {
		printk("BLE adv failed: %d\n", ret);
		return ret;
	}

	printk("BLE advertising as \"%s\"\n", CONFIG_BT_DEVICE_NAME);
	k_sem_give(&bt_ready_sem);
	return 0;
}
