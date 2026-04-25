# Настройка окружения с нуля — nRF52840 ProMicro / Nice!Nano

Инструкция для Ubuntu/Debian. Проверено на Ubuntu 25.10.

---

## Что будет установлено

- nRF Connect SDK v2.7.0 (включает Zephyr RTOS)
- Zephyr toolchain (arm-zephyr-eabi-gcc)
- west — менеджер пакетов Zephyr
- Кастомное описание платы promicro_nrf52840
- pyserial, screen — для работы с портом

---

## Шаг 1 — Системные зависимости

```bash
sudo apt update
sudo apt install -y \
    git cmake ninja-build gperf \
    ccache dfu-util device-tree-compiler wget \
    python3 python3-pip python3-setuptools python3-wheel \
    xz-utils file make gcc screen
```

---

## Шаг 2 — Установить west

west — это инструмент для управления Zephyr-проектами, аналог cargo/npm для Zephyr.

```bash
pip3 install --user west
```

Проверить:

```bash
west --version
# West version: v1.x.x
```

Если команда не найдена — добавьте `~/.local/bin` в PATH:

```bash
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## Шаг 3 — Установить nRF Connect SDK v2.7.0

Создаём рабочую папку:

```bash
mkdir -p ~/ncs
cd ~/ncs
```

Инициализируем workspace через west:

```bash
west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.7.0 v2.7.0
cd v2.7.0
west update
```

> Это займёт 10–30 минут — скачивается ~5 ГБ исходников Zephyr, nRF SDK, HAL и модулей.

После завершения установим Python-зависимости Zephyr:

```bash
pip3 install --user -r ~/ncs/v2.7.0/zephyr/scripts/requirements.txt
pip3 install --user -r ~/ncs/v2.7.0/nrf/scripts/requirements.txt
```

---

## Шаг 4 — Установить Zephyr toolchain (компилятор для ARM)

Скачиваем Zephyr SDK (содержит arm-zephyr-eabi-gcc):

```bash
cd ~/ncs
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz
tar xf zephyr-sdk-0.16.5_linux-x86_64.tar.xz -C ~/ncs/toolchains/
mv ~/ncs/toolchains/zephyr-sdk-0.16.5 ~/ncs/toolchains/e9dba88316
```

Устанавливаем udev-правила (нужны для доступа к USB без sudo):

```bash
cd ~/ncs/toolchains/e9dba88316
./setup.sh -t arm-zephyr-eabi
sudo cp sysroots/x86_64-pokysdk-linux/usr/share/zephyr/zephyr.udev /etc/udev/rules.d/50-zephyr.rules
sudo udevadm control --reload
```

---

## Шаг 5 — Создать env.sh

Этот файл настраивает переменные окружения для сборки. Создаём один раз:

```bash
cat > ~/ncs/env.sh << 'EOF'
export ZEPHYR_BASE=$HOME/ncs/v2.7.0/zephyr
export NRF_BASE=$HOME/ncs/v2.7.0/nrf
export PATH=$HOME/ncs/toolchains/e9dba88316/usr/local/bin:$HOME/ncs/toolchains/e9dba88316/bin:/usr/local/bin:$PATH
export LD_LIBRARY_PATH=$HOME/ncs/toolchains/e9dba88316/usr/local/lib:$LD_LIBRARY_PATH
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export GNUARMEMB_TOOLCHAIN_PATH=$HOME/ncs/toolchains/e9dba88316
source $ZEPHYR_BASE/zephyr-env.sh
EOF
```

Проверяем что всё работает:

```bash
source ~/ncs/env.sh
west --version       # должно показать версию
arm-zephyr-eabi-gcc --version  # должно показать версию компилятора
```

---

## Шаг 6 — Добавить кастомное описание платы

Nice!Nano совместима с форм-фактором Pro Micro. Описание платы не входит в стандартный Zephyr, нужно добавить вручную.

```bash
mkdir -p ~/ncs/custom_boards/boards/arm
cd ~/ncs/custom_boards/boards/arm
git clone https://github.com/vial-kb/vial-qmk promicro_nrf52840_src
# или скопировать папку promicro_nrf52840 с рабочей машины:
# scp -r user@old-machine:~/ncs/custom_boards/boards/arm/promicro_nrf52840 .
```

**Проще всего — скопировать папку с рабочей машины:**

```bash
# На новой машине:
scp -r user@old-machine:/home/devs/ncs/custom_boards ~/ncs/custom_boards
```

Структура должна быть такой:

```
~/ncs/custom_boards/
└── boards/
    └── arm/
        └── promicro_nrf52840/
            ├── CMakeLists.txt
            ├── Kconfig.board
            ├── Kconfig.defconfig
            ├── board.cmake
            ├── board.yml
            ├── promicro_nrf52840.yaml
            ├── promicro_nrf52840_nrf52840-pinctrl.dtsi
            ├── promicro_nrf52840_nrf52840.dts
            ├── promicro_nrf52840_nrf52840_common.dts
            └── promicro_nrf52840_nrf52840_defconfig
```

---

## Шаг 7 — Установить pyserial и screen

```bash
pip3 install pyserial
sudo apt install -y screen
```

---

## Шаг 8 — Скопировать проект

```bash
scp -r user@old-machine:/home/devs/CLionProjects/nrf ~/CLionProjects/nrf
# или git clone если проект в репозитории
```

---

## Шаг 9 — Первая сборка

```bash
source ~/ncs/env.sh
cd ~/CLionProjects/nrf
west build -b promicro_nrf52840/nrf52840 --build-dir build -- -DBOARD_ROOT=$HOME/ncs/custom_boards
```

Если сборка прошла успешно — появится файл `build/zephyr/zephyr.uf2`.

---

## Шаг 10 — Прошить плату

1. Подключить Nice!Nano по USB
2. Дважды нажать кнопку Reset — появится диск `NICENANO`
3. Скопировать прошивку:

```bash
cp build/zephyr/zephyr.uf2 /media/$USER/NICENANO/
```

4. Проверить вывод:

```bash
screen /dev/ttyACM0 115200
# Выход: Ctrl+A → K → Y
```

---

## Шаг 11 — Настроить dev.sh под новую машину

В файле `dev.sh` замените путь пользователя если он отличается от `devs`:

```bash
sed -i 's|/home/devs/|/home/ВАШ_ЮЗЕР/|g' ~/CLionProjects/nrf/dev.sh
```

Проверить и запустить:

```bash
chmod +x ~/CLionProjects/nrf/dev.sh
./dev.sh
```

---

## Возможные проблемы

### Нет доступа к /dev/ttyACM0
```bash
sudo usermod -aG dialout $USER
# перелогиниться
```

### west: unknown command "build"
Не выполнен `source ~/ncs/env.sh`. Выполните перед любой командой west.

### libpython3.x.so not found
```bash
export LD_LIBRARY_PATH=$HOME/ncs/toolchains/e9dba88316/usr/local/lib:$LD_LIBRARY_PATH
```
Это уже включено в `env.sh`.

### NICENANO не появляется
- Попробуйте двойной Reset быстрее
- Проверьте кабель (некоторые USB-кабели только для зарядки, без данных)
