# Контекст проекта — nice!nano + Zephyr

## Что это за проект

Прошивка для **nice!nano** (nRF52840): считывает данные IMU-датчика BNO085 по I2C и стримит их по BLE на Python-сервер `imu_server`. USB CDC ACM используется для консоли/shell.

Стек: Zephyr RTOS 3.6.99 / nRF Connect SDK v2.7.0, USB CDC ACM, BLE GATT Notify, I2C TWI, CEVA sh2 библиотека.

---

## Архитектура прошивки

### Файлы проекта
```
src/main.c          — BLE, main loop, GATT notify
src/imu.c           — поток imu_thread, sh2_open/config, mutex
src/bno085_hal.c    — HAL для sh2: I2C read/write/open/close
src/imu.h           — struct ImuData, imu_get_data()
src/bno085_hal.h    — extern sh2_Hal_t bno085_hal
src/sh2/            — CEVA sh2 библиотека (sh2.c/h, shtp.c/h, sh2_SensorValue.c/h, ...)
app.overlay         — переключение i2c0 на nordic,nrf-twi (TWI вместо TWIM)
prj.conf            — конфигурация Kconfig
CMakeLists.txt      — подключение всех .c файлов и include src/
```

### Потоки
| Поток | Стек | Приоритет | Что делает |
|-------|------|-----------|------------|
| `imu_tid` | 4096 | 5 | sh2_service() каждые 1 мс, обновляет latest ImuData |
| `watchdog_tid` | 1024 | 7 | каждые 5 с проверяет, не завис ли BLE в рекламе |
| main | — | — | imu_get_data() + bt_gatt_notify каждые 10 мс (notify каждый 5-й sample) |

### BLE GATT
- Service UUID:        `ceb5483e-36e1-4688-b7f5-ea07361b26a8`
- Characteristic UUID: `ceb5483e-36e1-4688-b7f5-ea07361b26a9` (NOTIFY)
- Device name: `LS_IMU_BLE`
- Payload: **18 байт** — 9 × int16, scale ×100 (гиро рад/с, аксел м/с², маг мкТл)

### I2C / BNO085
- Адрес: `0x4B` (PS0 подтянут к VCC на GY-BNO08x)
- Пины: SDA=P0.17, SCL=P0.20
- **Обязательно TWI** (не TWIM!) — BNO085 использует clock stretching при записи
- DTS overlay: `compatible = "nordic,nrf-twi"`, `clock-frequency = <I2C_BITRATE_STANDARD>`
- Overlay подключён через `set(DTC_OVERLAY_FILE ...)` в CMakeLists.txt

---

## BLE payload — кодирование int16 × 100

BlueZ на Linux согласовывает ATT MTU=23 (max payload 20 байт). Float-payload 36 байт не влезает → bt_gatt_notify возвращает -ENOMEM.

**Решение:** кодировать 9 float → 9 int16 (18 байт) со scale ×100.

nRF: `int16_t gx = (int16_t)(data.gx * 100.0f)`  
Сервер: `gx = unpacked[0] / 100.0`

Диапазоны при ×100:
- Гиро: ±327 рад/с (BNO085 max ±35) ✓
- Аксел: ±327 м/с² (BNO085 max ~40) ✓
- Маг: ±327 мкТл (типично ±100) ✓

---

## Известные нюансы

- **west не работает без `source /home/devs/ncs/env.sh`** — запускать в каждой новой сессии.
- **Pristine build после изменений Kconfig:** `west build --pristine`
- **BNO085 reset после sh2_open** — нормальное поведение; `event_cb` перехватывает `SH2_RESET` и вызывает `configure_sensors()` повторно.
- **imu_thread стартует с задержкой 5 с** — ждёт инициализации USB CDC, иначе printk теряется.
- **BlueZ фиксирует ATT MTU=23** — попытки согласовать MTU=64 через `bt_gatt_exchange_mtu` (с периферии) и `client.exchange_mtu(64)` (с Python) не помогают; BlueZ отвечает MTU=23.
- **Watchdog не спамит** — проверяет `current_conn != NULL` перед попыткой перезапустить рекламу.
- **`CONFIG_I2C_NRFX_TWI` не настраивается вручную** — выбирается автоматически из DTS compatible.
- **screen нельзя открывать пока dev.sh работает** — порт будет занят.
- **compile_commands.json в корне** — отфильтрованная копия из build/, обновляется dev.sh.

---

## Python-сервер imu_server

Путь: `/home/devs/PycharmProjects/imu_server`

Ключевые файлы:
- `src/core/app.py` — список датчиков (`SENSOR_JOINTS`), конфигурация UUID
- `src/bluetooth/ble_multy_client.py` — BLE клиент на bleak, scan → connect → start_notify
- `src/imu_buffer.py` — парсинг байт: `UNPACK_FORMAT = '<9h'`, `SCALE = 100.0`

Активный датчик:
```python
SensorJointConfig(
    sensor_name="LS_IMU_BLE",
    ble_address="ceb5483e-36e1-4688-b7f5-ea07361b26a9",  # UUID характеристики
    joint_name="knee_left",
    calib_subdir="sl_imu",
)
```

---

## Текущее состояние prj.conf

```
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_DEVICE_PRODUCT="Hello World"
CONFIG_USB_DEVICE_PID=0x0001
CONFIG_USB_CDC_ACM=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_UART_LINE_CTRL=y
CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT=n
CONFIG_BUILD_OUTPUT_UF2=y
CONFIG_SHELL=y
CONFIG_KERNEL_SHELL=y
CONFIG_SHELL_BACKEND_SERIAL=y
CONFIG_REBOOT=y
CONFIG_CBPRINTF_FP_SUPPORT=y

CONFIG_I2C=y

CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_DEVICE_NAME="LS_IMU_BLE"
CONFIG_BT_DEVICE_APPEARANCE=0
CONFIG_BT_MAX_CONN=1
CONFIG_BT_GATT_DYNAMIC_DB=n
CONFIG_BT_GATT_CLIENT=y
CONFIG_BT_L2CAP_TX_MTU=64
CONFIG_BT_BUF_ACL_TX_SIZE=72
CONFIG_BT_BUF_ACL_TX_COUNT=6
```
