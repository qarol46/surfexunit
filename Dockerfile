# Явно указываем платформу arm64 для Apple Silicon
FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-xacro \
    ros-jazzy-twist-mux \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rcl-interfaces \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-cv-bridge \
    python3-flask \
    python3-flask-cors \
    libopencv-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    x11-apps \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*
    
ARG USERNAME=osipovkirill
ARG USER_UID=501
ARG USER_GID=20

# Создаём группу и пользователя
RUN groupadd -g $USER_GID $USERNAME 2>/dev/null || true \
    && useradd -m -u $USER_UID -g $USER_GID -s /bin/bash $USERNAME 2>/dev/null || true \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Рабочая директория
ENV WS=/workspace
WORKDIR $WS

# Устанавливаем HOME
ENV HOME=/home/$USERNAME

# Создаём необходимые директории и устанавливаем права через UID/GID
RUN mkdir -p $HOME/.ros \
    && chown -R $USER_UID:$USER_GID $HOME

# Создаём runtime директорию
RUN mkdir -p /tmp/runtime-osipovkirill \
    && chown $USER_UID:$USER_GID /tmp/runtime-osipovkirill \
    && chmod 0700 /tmp/runtime-osipovkirill

ENV XDG_RUNTIME_DIR=/tmp/runtime-osipovkirill

# Настраиваем автоматическую загрузку окружения
RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc \
    && echo "source $WS/install/setup.bash" >> $HOME/.bashrc

# Переключаемся на нового пользователя
USER $USERNAME

CMD ["/bin/bash"]