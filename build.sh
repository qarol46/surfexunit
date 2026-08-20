#!/bin/bash
set -e

ARCH=$(uname -m)
IMAGE_NAME="surfexunit"

# Выбор базового образа в зависимости от архитектуры
if [ "$ARCH" = "aarch64" ]; then
    # Jetson Xavier NX (ARM64) - образ с поддержкой NVIDIA L4T
    # Примечание: при необходимости уточните тег (например, r35.4.1 или r36.2.0) под вашу версию JetPack
    BASE_IMAGE="dustynv/ros:humble-ros-base-l4t"
    echo "Обнаружена архитектура aarch64 (Jetson). Используем L4T образ: $BASE_IMAGE"
elif [ "$ARCH" = "x86_64" ]; then
    # Мини-ПК (x86_64) - стандартный десктопный образ ROS2
    BASE_IMAGE="osrf/ros:humble-desktop"
    echo "Обнаружена архитектура x86_64 (Mini-PC). Используем стандартный образ: $BASE_IMAGE"
else
    echo "Не поддерживаемая архитектура: $ARCH"
    exit 1
fi

# Возможность переопределить образ через переменную окружения
if [ -n "$CUSTOM_BASE_IMAGE" ]; then
    BASE_IMAGE=$CUSTOM_BASE_IMAGE
    echo "Образ переопределен пользователем: $BASE_IMAGE"
fi

echo "Начало сборки образа '$IMAGE_NAME'..."
docker build \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --build-arg ROS_DISTRO="humble" \
    -t "$IMAGE_NAME" \
    .

echo "Сборка завершена! Образ '$IMAGE_NAME' готов к использованию."