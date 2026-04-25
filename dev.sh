#!/bin/bash
set -e

source /home/devs/ncs/env.sh
cd /home/devs/CLionProjects/nrf

UF2=build/zephyr/zephyr.uf2

echo "=== Удаляю старый UF2 ==="
rm -f "$UF2"

echo "=== Сборка ==="
west build -b promicro_nrf52840/nrf52840 --build-dir build -- -DBOARD_ROOT=/home/devs/ncs/custom_boards

echo "=== Жду новый UF2 ==="
for i in $(seq 1 30); do
    [ -f "$UF2" ] && break
    sleep 1
done

if [ ! -f "$UF2" ]; then
    echo "ОШИБКА: UF2 не сгенерировался после сборки."
    exit 1
fi
echo "=== UF2 готов ==="

python3 - <<'EOF'
import json
with open('build/compile_commands.json') as f:
    db = json.load(f)
filtered = [e for e in db if '/home/devs/CLionProjects/nrf/' in e.get('file', '')]
with open('compile_commands.json', 'w') as f:
    json.dump(filtered, f, indent=2)
EOF

# Если плата уже в bootloader — сразу прошиваем
if [ -d /media/devs/NICENANO ]; then
    echo "=== Плата уже в bootloader, прошиваю... ==="
else
    # Освобождаем порт если занят
    if fuser /dev/ttyACM0 > /dev/null 2>&1; then
        echo "=== Порт занят, освобождаю... ==="
        fuser -k /dev/ttyACM0
        sleep 1
    fi

    echo "=== Отправляю команду dfu ==="
    python3 - <<'EOF'
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(0.5)
s.write(b'dfu\r\n')
time.sleep(0.5)
s.close()
EOF

    echo "=== Жду диск NICENANO ==="
    for i in $(seq 1 30); do
        [ -d /media/devs/NICENANO ] && break
        sleep 1
    done

    if [ ! -d /media/devs/NICENANO ]; then
        echo "ОШИБКА: диск NICENANO не появился. Попробуй двойной Reset вручную."
        exit 1
    fi
fi

echo "=== Прошивка ==="
cp "$UF2" /media/devs/NICENANO/

echo "=== Плата перезагружается, жду /dev/ttyACM0 ==="
for i in $(seq 1 15); do
    [ -e /dev/ttyACM0 ] && break
    sleep 1
done

echo "=== Открываю консоль ==="
screen /dev/ttyACM0 115200
