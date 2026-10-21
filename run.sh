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
        echo "Устройство ${symlink_path} не найдено. Пропускаем." >&2
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

# Если нужна графика (X11)
xhost +local:root >/dev/null 2>&1

# Применение udev правил для LiDAR
LIDAR_DIR="src/sllidar_ros2"
if [ -d "$LIDAR_DIR" ]; then
    echo "Применение udev правил для LiDAR..."
    (cd "$LIDAR_DIR" && sudo bash scripts/create_udev_rules.sh)
    echo "udev правила применены."
else
    echo "Директория $LIDAR_DIR не найдена. Пропускаем."
fi

# === ПРОВЕРКА И УВЕЛИЧЕНИЕ /dev/shm НА ХОСТЕ ===
# Поскольку мы используем --ipc=host, нужно увеличить shm на хосте
CURRENT_SHM=$(df -h /dev/shm | awk 'NR==2 {print $2}' | sed 's/G//')
if (( $(echo "$CURRENT_SHM < 6" | bc -l) )); then
    echo "/dev/shm на хосте меньше 6G ($CURRENT_SHM). Увеличиваем до 8G..."
    sudo mount -o remount,size=8G /dev/shm
    # Чтобы сохранить после перезагрузки:
    if ! grep -q "/dev/shm" /etc/fstab; then
        echo "tmpfs /dev/shm tmpfs defaults,size=8G 0 0" | sudo tee -a /etc/fstab
    fi
fi

echo "Запуск контейнера '$CONTAINER_NAME'..."

# Базовые аргументы Docker
DOCKER_ARGS=(
    -itd
    --name "$CONTAINER_NAME"
    --network host
    --ipc=host                  # Используем shm хоста
    --privileged
    # Увеличиваем shm-size на случай, если ipc=host не сработает
    --shm-size=6g               
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    -e DISPLAY="$DISPLAY"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "$PWD/src":/ros2_ws/src:rw
    -p 8765:8765
    --group-add dialout
    --memory=6g                
    --memory-swap=6g           
)

# === Явный проброс USB-устройств ===
DEV_LIDAR=$(get_real_device ttyLidar) && DOCKER_ARGS+=(--device="${DEV_LIDAR}:/dev/ttyLidar")
DEV_ESP1=$(get_real_device ttyESP1)   && DOCKER_ARGS+=(--device="${DEV_ESP1}:/dev/ttyESP1")
DEV_ESP2=$(get_real_device ttyESP2)   && DOCKER_ARGS+=(--device="${DEV_ESP2}:/dev/ttyESP2")
DEV_IMU=$(get_real_device ttyIMU)     && DOCKER_ARGS+=(--device="${DEV_IMU}:/dev/ttyIMU")

# Для RealSense D435
DOCKER_ARGS+=(--device=/dev/bus/usb:/dev/bus/usb)

# 1. Проброс сокета демона Argus (ОБЯЗАТЕЛЬНО для nvarguscamerasrc)
DOCKER_ARGS+=(-v /tmp/argus_socket:/tmp/argus_socket)

# 2. Явный проброс видеоустройств V4L2 (на всякий случай)
DOCKER_ARGS+=(--device=/dev/video0 --device=/dev/video1)

# === Поддержка GPU ===
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    DOCKER_ARGS+=(--runtime nvidia)
    echo "Jetson: используем --runtime nvidia для доступа к GPU."
elif command -v nvidia-smi >/dev/null 2>&1; then
    DOCKER_ARGS+=(--gpus all)
    echo "x86_64: используем --gpus all."
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME"

echo ""
echo "Контейнер '$CONTAINER_NAME' успешно запущен."
echo "Размер /dev/shm внутри контейнера:"
docker exec "$CONTAINER_NAME" df -h /dev/shm
echo "Используйте './exec.sh' для входа в контейнер."