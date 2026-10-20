# imu_waveshare_ros2_driver
10 DOF ROS IMU (A) is a 10-axis IMU sensor that integrates an ARM 32-bit DSP processor, BLE5.3 Bluetooth, high-precision accelerometer, gyroscope, magnetometer, and temperature and pressure sensor. When used together, it can collect surrounding acceleration, gyroscope, and magnetic field data, and detect the motion attitude of the robot.


## Инструкция по установке

1. Установка библиотеки под serial-port

```bash
cd serial && mkdir build && cd build
cmake .. && make
sudo make install
```

2. Установка остальных зависимостей

```bash
sudo apt update && sudo apt install libserial-dev  ros-${ROS_DISTRO}-imu-tools
```

3. Сборка основного пакета

```bash
colcon build --packages-select imu
```

### Подключение устройства

1. Определите устройство:

```bash
ls /dev/ttyUSB*
```

2. Настройте права:

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
newgrp dialout
```