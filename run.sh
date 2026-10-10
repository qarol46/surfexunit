#!/bin/bash
set -e

CONTAINER_NAME="surfexunit_container"
IMAGE_NAME="surfexunit"

# Функция для получения реального пути устройства по symlink
get_real_device() {
    local symlink_name="$1"
    local symlink_path="/dev/${symlink_name}"
    local real_path
    real_path=$(readlink -f "$symlink_path" 2>/dev/null || true)
    
    if [ -z "$real_path" ] || [ ! -e "$real_path" ]; then
        echo "⚠️  Устройство ${symlink_path} не найдено. Пропускаем." >&2
        return 1
    fi
    echo "$real_path"
}

# Остановка и удаление старого контейнера
if [ "$(docker ps -aq -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Остановка и удаление существующего контейнера '$CONTAINER_NAME'..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1
    docker rm "$CONTAINER_NAME" >/dev/null 2>&1
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
echo "ROS_DOMAIN_ID установлен в: $ROS_DOMAIN_ID"

# Если нужна графика (X11) - раскомментируйте эти строки
xhost +local:root >/dev/null 2>&1

echo "Запуск контейнера '$CONTAINER_NAME'..."

# Базовые аргументы Docker
DOCKER_ARGS=(
    -itd
    --name "$CONTAINER_NAME"
    --network host
    --ipc=host
    --privileged
    --shm-size=2G
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    -e DISPLAY="$DISPLAY"                    # Раскомментируйте, если нужна графика
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw      # Раскомментируйте, если нужна графика
    -v "$PWD/src":/ros2_ws/src:rw
    --group-add dialout
)

# === Явный проброс USB-устройств ===
DEV_LIDAR=$(get_real_device ttyLidar) && DOCKER_ARGS+=(--device="${DEV_LIDAR}:/dev/ttyLidar")
DEV_ESP1=$(get_real_device ttyESP1)   && DOCKER_ARGS+=(--device="${DEV_ESP1}:/dev/ttyESP1")
DEV_ESP2=$(get_real_device ttyESP2)   && DOCKER_ARGS+=(--device="${DEV_ESP2}:/dev/ttyESP2")
DEV_IMU=$(get_real_device ttyIMU)     && DOCKER_ARGS+=(--device="${DEV_IMU}:/dev/ttyIMU")

# Для RealSense D435 критически важен доступ к USB-шине (librealsense использует libusb)
DOCKER_ARGS+=(--device=/dev/bus/usb:/dev/bus/usb)

# === Поддержка GPU ===
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    DOCKER_ARGS+=(--runtime nvidia)
    echo "🚀 Jetson: используем --runtime nvidia для доступа к GPU."
elif command -v nvidia-smi >/dev/null 2>&1; then
    DOCKER_ARGS+=(--gpus all)
    echo "🎮 x86_64: используем --gpus all."
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME"

echo ""
echo "Контейнер '$CONTAINER_NAME' успешно запущен."
echo "Используйте './exec.sh' (или './new_window_cont.sh') для открытия нового окна терминала внутри контейнера."
