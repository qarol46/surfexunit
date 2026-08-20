# Базовый образ передается через аргумент при сборке
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM ${BASE_IMAGE}

# Отключаем интерактивные запросы при установке пакетов
ENV DEBIAN_FRONTEND=noninteractive

# Версия ROS дистрибутива (по умолчанию humble)
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}

# Установка системных и ROS2 зависимостей на основе ваших package.xml
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-vision-opencv \
    libopencv-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Инициализация rosdep
RUN rosdep init || true
RUN rosdep update

# Создание рабочего пространства
ENV WS_DIR=/ros2_ws
WORKDIR ${WS_DIR}

# Копирование исходного кода (будет перезаписано при монтировании тома в run.sh для удобной разработки)
COPY src/ src/

# Установка зависимостей через rosdep для пакетов в workspace
RUN rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y || true

# Сборка рабочего пространства
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install"

# Автоматическая подгрузка окружения при запуске bash
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source ${WS_DIR}/install/setup.bash" >> /root/.bashrc

# Команда по умолчанию
CMD ["/bin/bash"]