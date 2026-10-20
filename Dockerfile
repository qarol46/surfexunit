ARG BASE_IMAGE=tiryoh/ros2-desktop-vnc:humble-arm64-20230129T1546 
FROM ${BASE_IMAGE}

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV ROS_DISTRO=humble

# Исправление GPG-ключа ROS
RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# 1. Установка основных системных и ROS2 зависимостей
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
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    libopencv-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Установка python3-serial
RUN apt-get update && apt-get install -y \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# 3. Установка Foxglove и базовых интерфейсов
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-humble-rcl-interfaces \
    ros-humble-std-srvs \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*
    
# 4. Установка image_transport и topic_tools (для throttle и сжатия)
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# 5. Установка libserial и imu-tools
RUN apt-get update && apt-get install -y \
    libserial-dev \
    ros-${ROS_DISTRO}-imu-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

ENV WS_DIR=/ros2_ws
WORKDIR ${WS_DIR}

COPY src/ src/

# rosdep установит системные зависимости для вашего кода
RUN rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y || true

# Этап 1: Собираем пакет serial первым (игнорируем ошибку, если пакета нет)
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select serial --parallel-workers 2 --symlink-install || true"

# Этап 2: Подгружаем собранный serial и собираем все остальные пакеты
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WS_DIR}/install/setup.bash && \
    colcon build --packages-skip serial --parallel-workers 2 --symlink-install"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source ${WS_DIR}/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#ENTRYPOINT ["/entrypoint.sh"]
CMD ["entrypoint.sh"]