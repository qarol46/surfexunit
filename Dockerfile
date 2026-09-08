ARG BASE_IMAGE=tiryoh/ros2-desktop-vnc:humble-arm64-20230129T1546 
FROM ${BASE_IMAGE}

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV ROS_DISTRO=humble

RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# Установка системных и ROS2 зависимостей, включая пакеты для Intel RealSense
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

RUN rosdep init || true
RUN rosdep update

ENV WS_DIR=/ros2_ws
WORKDIR ${WS_DIR}

COPY src/ src/

# rosdep установит системные зависимости для вашего кода. 
# Он проигнорирует отсутствие ros-humble-realsense2-camera в apt, 
# так как мы соберем его из исходников на следующем шаге.
RUN rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y || true

# Сборка workspace (здесь соберется и realsense-ros, если он лежит в папке src/)
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source ${WS_DIR}/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
