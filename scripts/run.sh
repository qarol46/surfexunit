#!/bin/bash

# Получаем IP хоста
HOST_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1)

# Удаляем старый контейнер
docker rm -f surfexunit_sim_jazzy 2>/dev/null

# Разрешаем подключения
xhost +$HOST_IP 2>/dev/null
xhost +localhost 2>/dev/null

# Запускаем контейнер
docker run -it \
  --name surfexunit_sim_jazzy \
  --volume "$(pwd)":/workspace \
  --publish 8765:8765 \
  --publish 5001:5000 \
  --env="DISPLAY=$HOST_IP:0" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --env="XDG_RUNTIME_DIR=/tmp/runtime-osipovkirill" \
  --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  surfexunit_sim:jazzy