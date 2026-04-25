# Инструкция по сборке и прошивке nice!nano

## Плата
- **Модель:** nice!nano (nRF52840)
- **Bootloader:** UF2 Bootloader 0.9.2
- **Zephyr SDK:** v2.7.0
- **Board:** `promicro_nrf52840/nrf52840`

---

## Быстрый старт — весь цикл одной командой

```bash
source /home/devs/ncs/env.sh
cd /home/devs/CLionProjects/nrf
./dev.sh
```

Скрипт сам соберёт, отправит команду `dfu`, дождётся диска NICENANO, прошьёт и откроет консоль.

---

## Ручной цикл по шагам

### 1. Активировать окружение (один раз в сессии терминала)

```bash
source /home/devs/ncs/env.sh
cd /home/devs/CLionProjects/nrf
```

### 2. Редактировать код

```
src/main.c
```

### 3. Собрать прошивку

```bash
west build -b promicro_nrf52840/nrf52840 --build-dir build -- -DBOARD_ROOT=/home/devs/ncs/custom_boards
```

Результат: `build/zephyr/zephyr.uf2` (генерируется автоматически благодаря `CONFIG_BUILD_OUTPUT_UF2=y`)

### 4. Перевести плату в режим bootloader

**Вариант A — программно через shell (если плата запущена):**

```bash
screen /dev/ttyACM0 115200
# В консоли ввести:
dfu
# Выйти из screen: Ctrl+A → K → Y
```

**Вариант B — вручную:**

Дважды быстро нажать кнопку Reset. Светодиод начнёт пульсировать, появится диск `NICENANO`.

### 5. Прошить

```bash
cp build/zephyr/zephyr.uf2 /media/devs/NICENANO/
```

Плата автоматически перезагрузится.

### 6. Открыть консоль

```bash
screen /dev/ttyACM0 115200
```

Выход: **Ctrl+A → K → Y**

---

## Структура проекта

```
nrf/
├── src/main.c              # Основной код
├── prj.conf                # Конфигурация Zephyr (Kconfig)
├── CMakeLists.txt          # CMake (не менять)
├── dev.sh                  # Скрипт полного цикла
├── compile_commands.json   # Для CLion (обновляется dev.sh)
└── build/
    └── zephyr/
        ├── zephyr.hex
        └── zephyr.uf2
```

---

## Как работает команда dfu

В `main.c` зарегистрирована shell-команда `dfu`. При вызове она:
1. Записывает магическое число `0x57` в регистр `GPREGRET`
2. Вызывает `sys_reboot(SYS_REBOOT_COLD)`

Bootloader при старте читает `GPREGRET`, видит `0x57` и остаётся в режиме UF2 вместо запуска приложения.

---

## Пути окружения

| Что | Путь |
|-----|------|
| env.sh | `/home/devs/ncs/env.sh` |
| Zephyr | `/home/devs/ncs/v2.7.0/zephyr` |
| Toolchain | `/home/devs/ncs/toolchains/e9dba88316` |
| Custom boards | `/home/devs/ncs/custom_boards` |
