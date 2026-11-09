#!/bin/bash

# Директория для сохранения карт
MAP_DIR="/home/kirill/ros2_ws/src/surfexunit/src/asump_localization/maps"
DB_PATH="/home/kirill/.ros/rtabmap.db"

# Проверка аргументов
if [ $# -eq 0 ]; then
    echo "Ошибка: не указано название карты"
    echo "Использование: $0 <название_карты>"
    echo "Пример: $0 warehouse_map"
    exit 1
fi

MAP_NAME=$1
MAP_2D_PATH="${MAP_DIR}/${MAP_NAME}"
MAP_3D_PATH="${MAP_DIR}/${MAP_NAME}.pcd"

# Создание директории, если она не существует
if [ ! -d "$MAP_DIR" ]; then
    echo "Создание директории: $MAP_DIR"
    mkdir -p "$MAP_DIR"
fi

# Проверка, существуют ли уже файлы с таким именем
if [ -f "${MAP_2D_PATH}.pgm" ] || [ -f "${MAP_3D_PATH}" ]; then
    echo "Предупреждение: файлы карты с именем '${MAP_NAME}' уже существуют."
    read -p "Перезаписать? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Сохранение отменено."
        exit 1
    fi
fi

echo "========================================="
echo "Начинается сохранение карты: ${MAP_NAME}"
echo "========================================="

# ---------------------------------------------------------
# ШАГ 1: Сохранение 2D карты (Occupancy Grid для Nav2)
# ---------------------------------------------------------
echo "[1/2] Сохранение 2D карты (PGM + YAML)..."
ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_2D_PATH" \
    --ros-args \
    -p save_map_timeout:=10.0 \
    -p free_thresh:=0.25 \
    -p occupied_thresh:=0.65

if [ $? -ne 0 ]; then
    echo "✗ Ошибка при сохранении 2D карты."
    echo "Убедитесь, что RTAB-Map запущен и публикует топик /map."
    exit 1
fi
echo "✓ 2D карта успешно сохранена."

echo "========================================="
echo "Готово! Файлы сохранены в: $MAP_DIR"
echo "  - ${MAP_NAME}.pgm  (2D карта для навигации)"
echo "  - ${MAP_NAME}.yaml (Метаданные 2D карты)"
echo "========================================="