#ifndef BLE_H
#define BLE_H

#include <stdbool.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

extern bool notify_enabled;
extern struct bt_conn *current_conn;

int ble_init(const struct bt_data *ad, size_t ad_len,
             const struct bt_data *sd, size_t sd_len);

#endif /* BLE_H */
