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
    -v "$PWD/src":/surfexunit_ws/src:rw
)

# Добавление поддержки GPU в зависимости от архитектуры
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    # Jetson (L4T) - используем NVIDIA Container Runtime
    DOCKER_ARGS+=(--runtime nvidia)
    echo "Обнаружен Jetson. Используем --runtime nvidia для доступа к GPU."
elif command -v nvidia-smi >/dev/null 2>&1; then
    # Мини-ПК с NVIDIA GPU - используем стандартный флаг --gpus
    DOCKER_ARGS+=(--gpus all)
    echo "Обнаружена NVIDIA GPU на x86_64. Используем --gpus all."
fi

# Поскольку вы используете образ tiryoh/ros2-desktop-vnc, 
# добавляем проброс порта для VNC (по умолчанию 6080 для веб-доступа или 5900 для VNC-клиента)
DOCKER_ARGS+=(
    -p 6080:6080
    -p 5900:5900
)

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME"

echo "Контейнер '$CONTAINER_NAME' успешно запущен."
echo ""
echo "========================================="
echo "ДОСТУП К ГРАФИЧЕСКОМУ ИНТЕРФЕЙСУ (VNC):"
echo "Веб-браузер: http://$(hostname -I | awk '{print $1}'):6080"
echo "VNC-клиент:  $(hostname -I | awk '{print $1}'):5900"
echo "========================================="
echo ""
echo "Используйте './exec.sh' для открытия нового окна терминала внутри контейнера."
