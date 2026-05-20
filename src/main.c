#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <string.h>
#include "imu.h"
#include "transport.h"
#include "ble.h"

/* ── Частоты ─────────────────────────────────────────────────────────────── */
#define SAMPLE_HZ   40U   /* частота опроса датчика (должна совпадать с icm45686.c) */
#define BLE_HZ       5U   /* частота BLE-нотификаций                               */
#define LOG_HZ       1U   /* частота вывода в консоль (0 = выключить)              */

/* Physical-unit scaling for BLE int16 payload:
 *   Gyro:  rad/s  × 1000  →  mrad/s   (max ±34.9 rad/s → ±34900, safe with bad-frame filter at ±29000 raw)
 *   Accel: m/s²   × 1000  →  mm/s²    (max ±2g = ±19620, fits int16)
 *   Mag:   µT     × 100   →  0.01 µT  (max ±60 µT = ±6000, fits int16)
 */
#define IMU_GYRO_SCALE  1000.0f   /* rad/s  → int16 */
#define IMU_ACCEL_SCALE 1000.0f   /* m/s²   → int16 */
#define IMU_MAG_SCALE    100.0f   /* µT     → int16 */

/* IMU payload — 9 int16, 18 bytes. Fits in default ATT MTU=23 (max payload 20). */
typedef struct __attribute__((packed)) {
	int16_t gx, gy, gz;
	int16_t ax, ay, az;
	int16_t mx, my, mz;
} ImuPayload;

/* Service UUID:        ceb5483e-36e1-4688-b7f5-ea07361b26a8 */
/* Characteristic UUID: ceb5483e-36e1-4688-b7f5-ea07361b26a9 */
#define IMU_SVC_UUID_VAL \
	BT_UUID_128_ENCODE(0xceb5483e, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26a8)
#define IMU_CHR_UUID_VAL \
	BT_UUID_128_ENCODE(0xceb5483e, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26a9)

static struct bt_uuid_128 imu_svc_uuid = BT_UUID_INIT_128(IMU_SVC_UUID_VAL);
static struct bt_uuid_128 imu_chr_uuid = BT_UUID_INIT_128(IMU_CHR_UUID_VAL);

static ImuPayload imu_payload;

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("Notify %s\n", notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(imu_svc,
	BT_GATT_PRIMARY_SERVICE(&imu_svc_uuid),
	BT_GATT_CHARACTERISTIC(&imu_chr_uuid.uuid,
		BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_NONE,
		NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, IMU_SVC_UUID_VAL),
};

static void imu_log(const ImuPayload *data, uint32_t dt_us, uint32_t *cnt)
{
	if (LOG_HZ == 0 || ++(*cnt) < (SAMPLE_HZ / LOG_HZ)) {
		return;
	}
	*cnt = 0;
	uint32_t hz = (dt_us > 0) ? (1000000U / dt_us) : 0;
	printk("[dt=%uus ~%uHz] G:%.2f %.2f %.2f A:%.2f %.2f %.2f M:%.2f %.2f %.2f\n",
	       dt_us, hz,
	       (double)data->gx, (double)data->gy, (double)data->gz,
	       (double)data->ax, (double)data->ay, (double)data->az,
	       (double)data->mx, (double)data->my, (double)data->mz);
}

int main(void)
{
	int ret = transport_init();
	if (ret != 0) {
		while (1) { k_sleep(K_MSEC(1000)); }
	}

	ret = ble_init(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret != 0) {
		while (1) { k_sleep(K_MSEC(1000)); }
	}

	uint32_t log_cnt = 0;
	uint32_t notify_cnt = 0;
	uint32_t prev_ts_us = 0;

	while (1) {
		ImuData data;
		if (imu_get_data(&data)) {
			imu_payload.gx = (int16_t)(data.gx * IMU_GYRO_SCALE);
			imu_payload.gy = (int16_t)(data.gy * IMU_GYRO_SCALE);
			imu_payload.gz = (int16_t)(data.gz * IMU_GYRO_SCALE);
			imu_payload.ax = (int16_t)(data.ax * IMU_ACCEL_SCALE);
			imu_payload.ay = (int16_t)(data.ay * IMU_ACCEL_SCALE);
			imu_payload.az = (int16_t)(data.az * IMU_ACCEL_SCALE);
			imu_payload.mx = (int16_t)(data.mx * IMU_MAG_SCALE);
			imu_payload.my = (int16_t)(data.my * IMU_MAG_SCALE);
			imu_payload.mz = (int16_t)(data.mz * IMU_MAG_SCALE);

			uint32_t dt_us = (prev_ts_us != 0) ? (data.timestamp_us - prev_ts_us) : 0;
			prev_ts_us = data.timestamp_us;

			imu_log(&imu_payload, dt_us, &log_cnt);

			if (notify_enabled && (++notify_cnt % (SAMPLE_HZ / BLE_HZ)) == 0) {
				const struct bt_gatt_attr *attr = &imu_svc.attrs[2];
				bt_gatt_notify(current_conn, attr, &imu_payload, sizeof(imu_payload));
			}
		}
		k_sleep(K_MSEC(1000U / (SAMPLE_HZ * 5U)));
	}

	return 0;
}
