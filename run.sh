#!/bin/bash
set -e

CONTAINER_NAME="surfexunit_container"
IMAGE_NAME="surfexunit"

# Остановка и удаление старого контейнера с тем же именем
if [ "$(docker ps -aq -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Остановка и удаление существующего контейнера '$CONTAINER_NAME'..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1
    docker rm "$CONTAINER_NAME" >/dev/null 2>&1
fi

# Настройка ROS_DOMAIN_ID (важно для коммуникации между устройствами)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
echo "ROS_DOMAIN_ID установлен в: $ROS_DOMAIN_ID"

# Разрешение локальному root доступ к X-серверу для отображения rviz2
xhost +local:root >/dev/null 2>&1

echo "Запуск контейнера '$CONTAINER_NAME'..."

# Базовые аргументы Docker
DOCKER_ARGS=(
    -itd
    --name "$CONTAINER_NAME"
    --network host
    --ipc=host
    --privileged
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    -e DISPLAY="$DISPLAY"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v /dev:/dev
    # Монтирование исходников позволяет редактировать код на хосте и пересобирать его внутри контейнера
    -v "$PWD/src":/ros2_ws/src:rw
)

# Добавление поддержки GPU, если это Jetson или есть NVIDIA GPU на мини-ПК
if uname -m | grep -q "aarch64" || command -v nvidia-smi >/dev/null 2>&1; then
    DOCKER_ARGS+=(--gpus all)
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME"

echo "Контейнер '$CONTAINER_NAME' успешно запущен."
echo "Используйте './exec.sh' для открытия нового окна терминала внутри контейнера."