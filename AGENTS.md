# AGENTS.md — surfexunit

## Обзор проекта

Это ROS 2 (Jazzy) workspace автономного робота-уборщика склада «asump» (surfex_robot): колёсная платформа с дифференциальным приводом (4 ведущих колеса), лидар, ультразвуковые датчики, RealSense-камера, ИМУ. Репозиторий — корень colcon-workspace: исходники лежат в `src/`, артефакты сборки — в `build/`, `install/`, `log/` (все три в `.gitignore` и в репозиторий не коммитятся).

Основной сценарий: робот ездит по заранее построенной карте склада, локализуется (slam_toolbox в режиме localization + собственный инициализатор позы на частицах), строит маршруты уборки (mission_planner/mission_executor), следует по ним (path_follower) и выдаёт скорости на микроконтроллер ESP32 через UART. Управление также возможно через веб-интерфейс телеуправления. Есть симуляция в Gazebo Harmonic (ros_gz) со складским миром.

Язык комментариев и документации в коде — преимущественно русский, коммит-месседжи — на английском (`upd: ...`).

## Структура репозитория

```
scripts/uart_sender.py  # автономный отладочный скрипт отправки скоростей на ESP32 по UART
src/                    # ROS 2 пакеты (10 штук, см. ниже)
build/ install/ log/    # артефакты colcon (в .gitignore, не редактировать)
```

## Пакеты и их роли

### C++ (ament_cmake)

- **`asump_localization`** — локализация и построение карт. Ноды:
  - `lidar_odometry_node` (`src/lidar_odometry.cpp`) — одометрия по лидару (PCL).
  - `rgbd_mapper_node` (`src/rgbd_mapper.cpp`) — 3D-картирование по RGB-D (PCL, сохраняет `rgbd_map.pcd`).
  - `pose_initializer_node` (`src/pose_initializer.cpp`) — инициализация позы на карте фильтром частиц (particle filter + scan matching; OpenCV, yaml-cpp, Eigen). Публикует `/initialpose`, сервис `initialize_pose`.
  - Собственные сервисы (`srv/`): `InitializePose.srv`, `GetPathToPoint.srv`, `GetWaypoint.srv` — генерируются через `rosidl_generate_interfaces`, зависят от `geometry_msgs`/`nav_msgs`. Другие пакеты используют их через `<depend>asump_localization</depend>`.
  - `config/`: `slam_localization_params.yaml`, `slam_mapper_params.yaml` (slam_toolbox), `amcl.yaml` (экспериментальный переход на AMCL), `ekf.yaml`, rviz-конфиги.
  - `maps/`: готовые карты (`warehouse_map`, `new_warehouse_map` — pgm+yaml, `rgbd_map.pcd`).
- **`asump_nav`** — навигация. Ноды:
  - `path_follower_node` (+ shared-библиотека `path_follower_core`, заголовки в `include/asump_nav/`) — чистое следование по пути (pure pursuit со сглаживанием скоростей), параметры в `config/path_follower.yaml`, путь запрашивает через сервис `get_path_to_point`.
  - `mission_planner_node` — строит покрывающий маршрут уборки по карте, выдаёт waypoints через `GetWaypoint`.
  - `mission_executor_node` — оркестратор автономной уборки: action-клиент `nav2_msgs/NavigateToPose` + сервис `GetWaypoint`.
  - `obstacle_safety_gate_node` — аварийный «стоп-вентиль» по лидару/карте, публикует в `cmd_vel_safe` (высший приоритет twist_mux).
  - `dock_controller_node` — докование на зарядную станцию: сервис `std_srvs/Trigger` `dock` запускает двухфазный заезд — (1) подход через action `navigate_to_pose` (path_follower) в точку перед пандусом задом к станции (точка считается от позы `/zones/charger_pose` + параметры `ramp_offset`/`standoff_dist`), (2) реверс с P-выравниванием по ArUco-маркеру (DICT_4X4_50 id 0) в `/back_camera/image` (OpenCV), финиш по задним УЗ `/us/back_left|right` (`us_stop_distance`). Параметры `config/docking.yaml`, запуск `launch/docking.launch.py`, отладочная картинка в `/dock/debug_image`.

### Python (ament_python, setup.py)

- **`asump_control`** — `velocity_forward_node` (`asump_control/asump_control.py`): подписывается на `/cmd_vel`, пересчитывает в скорости 4 колёс (`fl/rl/fr/rr_wheel_joint`) и отправляет бинарный пакет на ESP32 по UART (`serial_port`, default `/dev/ttyUSB0`, 115200). Также обслуживает `/aux_motor_cmd`, `/stepper_cmd`, сервис `/set_aux_motor`. Требует `pyserial` (не заявлен в package.xml — ставится вручную/в образе).
- **`ultrasonic_bridge`** — `ultrasonic_node`: читает 8 ультразвуковых датчиков с UART (кадр `0xAA + 32 байта payload (8×float LE) + XOR-checksum + 0x55`), публикует `sensor_msgs/Range` с `frame_id` из параметра `frame_ids`.
- **`map_tools`** — инструменты карты: `map_publisher` (публикация occupancy-карты из yaml/pgm в `/map_base`), `inflation_publisher` (инфляция препятствий), `zones_publisher` (зоны, поза зарядной станции `/zones/charger_pose`), `path_planner` (A* по occupancy grid, сервис `GetPathToPoint`). Конфиг `config/map_tools.yaml`, запуск `launch/map_tools.launch.py`.
- **`asump_web_teleop`** — `web_teleop_node`: Flask-сервер на `0.0.0.0:5000` (зависимости `python3-flask`, `python3-flask-cors`), веб-джойстик, публикует `/cmd_vel`, показывает `/scan`, кнопка сервиса `/set_aux_motor`.

### Описание, симуляция, bringup

- **`asump_description`** — полный URDF/xacro робота (`urdf/robot.xacro` включает `base_link`, `ros2_control`, `gazebo_control`, `lidar`, `imu`, камеры `d415`/`depth_camera`/`side_camera`), меши STL.
- **`asump_description2`** — упрощённое/новое описание (`urdf/robot.xacro` + `gazebo_control.xacro`; в DiffDrive-плагине задан `max_wheel_torque` 2.5 Н·м — без ограничения момента робот «вкручивался» в вертикальные стены), которое фактически используется симуляцией (см. `_gazebo_resource_path` в `asump_sim/launch/launch_sim.launch.py`, подключает также `realsense2_description`). Меши STL хранятся уже повёрнутыми в Z-up (компенсирующий `rpy` в visual/collision запечён в геометрию — иначе Foxglove отображал детали с неверной ориентацией; у STL нет метаданных об оси).
- **`asump_sim`** — Gazebo Harmonic (ros_gz): миры `worlds/{warehouse,tugbot_warehouse,empty}.sdf` (в `warehouse.sdf` шаг физики 4 мс вместо 1 мс — чтобы real_time_factor не проседал под нагрузкой GUI+RViz; просадка RTF давала эффект «невесомости» при падении), модели в `models/`. Складские модели — собственные, собранные из SDF-примитивов (`warehouse_shelf`, `warehouse_pallet`, `warehouse_pallet_jack`, `warehouse_box_cluster`, `warehouse_floor_marking`, `warehouse_trash_bin`) + `charging_dock` (зарядная станция: визуал и коллизия — из STL `models/charging_dock/meshes/charging_dock.stl`, коллизия-меш позволяет роботу заезжать на пандус + скользкая оболочка-коробка вокруг башни (mu=0.05, чтобы колёса не «вкручивались» в её вертикальную стену); на грани перед спуском — ArUco-маркер 20×20 см, DICT_4X4_50 id 0, текстура `materials/textures/aruco_4x4_50_id0.png`; включается в `warehouse.sdf` через `model://charging_dock`, стоит примерно в центре сцены (центр меша в x=0, y=-2.5), пандусом на +x). Пока эти локальные модели использует только `charging_dock` — остальное `warehouse.sdf` и `tugbot_warehouse.sdf` тянут с Fuel по URL (`fuel.ignitionrobotics.org`/`fuel.gazebosim.org`), поэтому при первом запуске нужна сеть (далее — кэш `~/.gz/fuel`). Мост `config/ros_gz_bridge.yaml` (важно: `gz_topic_name` должны совпадать с `<topic>` сенсоров в `asump_description2/urdf/robot.xacro` — gz-топики ультразвука `*_us/scan`, камер `*_camera/color|depth/...`; облаков точек нет, т.к. сенсоры depth_camera, а не rgbd_camera; для всех сенсорных топиков задан `override_frame_id` — Gazebo шлёт scoped-фреймы `asump0/...`, которых нет в URDF-TF-дереве, поэтому frame_id подменяется на имена линков из URDF; ультразвуковые сенсоры — `gpu_lidar`, для них в xacro заведены scoped-линки `asump0/base_footprint/*_sensor`). `launch/launch_sim.launch.py`: robot_state_publisher + Gazebo (`-r`, автостарт, с GUI) + спавн робота `asump0` (x=2.0, y=-1.5, z=0.3, Y=-2.68 — рядом с зарядной станцией, ~2.24 м от её центра, лицом к ней) + ros_gz_bridge + RViz (`config/default.rviz`); ros2_control-контроллеры (`joint_broad`, `diff_drive`, спавн с задержкой 6 с) сейчас закомментированы. Аргументы: `use_sim_time` (default true), `use_rviz_arg` (default true — на нативной Ubuntu RViz работает; `use_rviz_arg:=false` отключает).
- **`asump_bringup`** — верхнеуровневый запуск:
  - `launch/asump_bringup.launch.py` — полный стек: sim → localization → path_planner → twist_mux → navigation.
  - `launch/mapping_bringup.launch.py` — режим построения карты.
  - `config/twist_mux.yaml` — мультиплексор скоростей в `cmd_vel`: `cmd_vel_safe` (estop, приоритет 300) > `cmd_vel_teleop` (200) > `cmd_vel_nav` (100).

## Сборка и запуск

Сборка нативно на Ubuntu (ROS 2 Jazzy и Gazebo Harmonic установлены на хосте):

```bash
colcon build --symlink-install          # полная сборка
colcon build --symlink-install --packages-select asump_nav asump_localization   # точечная сборка пакетов
source install/setup.bash
```

Точки входа:

```bash
ros2 launch asump_bringup asump_bringup.launch.py      # полный стек (сим + локализация + нав)
ros2 launch asump_sim launch_sim.launch.py                 # симуляция + RViz (use_rviz_arg:=false — без RViz)
ros2 launch asump_bringup mapping_bringup.launch.py    # режим картирования (slam_toolbox mapper)
ros2 launch map_tools map_tools.launch.py              # только карта/планировщик
ros2 launch asump_localization localization.launch.py map_yaml_file:=/abs/path/map.yaml
ros2 run asump_web_teleop web_teleop_node              # веб-телеуправление, http://<ip>:5000
ros2 run asump_control velocity_forward_node           # UART-мост к ESP32 (нужен /dev/ttyUSB0)
ros2 run ultrasonic_bridge ultrasonic_node
```

Известный подводный камень: в `asump_localization/launch/localization.launch.py` дефолт `map_yaml_file` захардкожен на абсолютный путь `/home/kirill/ros2_ws/...` — при переносе передавайте аргумент явно.

## Соглашения по коду

- C++17, флаги `-Wall -Wextra -Wpedantic` (заданы в CMakeLists обоих C++ пакетов). Стандартные ROS-идиомы: `rclcpp`, параметры через `declare_parameter`, неймспейсы (`namespace autonomous_navigation` в asump_nav).
- Python-пакеты собраны по шаблону ament_python: `setup.py` + `setup.cfg` + `resource/`, точки входа — `console_scripts` в `setup.py`. Для запуска используются имена из `entry_points`, а не файлы.
- Комментарии пишутся на русском, логи нод — в основном на английском.
- Ключевые фреймы: `map` → … → `base_footprint` → `base_link`. Ключевые топики: `/cmd_vel` (после twist_mux), `/cmd_vel_nav`, `/cmd_vel_teleop`, `/cmd_vel_safe`, `/scan`, `/map_base`, `/initialpose`, `/joint_states`, `/zones/charger_pose`.
- Аппаратный протокол ESP32 (важно не менять без синхронизации с прошивкой): отправка `'<ffBB'` (vx, vz float LE, aux_motor, stepper) + XOR-checksum; приём ультразвука — кадр `0xAA … 0x55` с 8 float'ами.

## Тестирование

Юнит-тестов по существу нет. В каждом Python-пакете — только шаблонные линтеры ament (`test/test_flake8.py`, `test_pep257.py`, `test_copyright.py`), запускаются через:

```bash
colcon test && colcon test-result --verbose
```

При добавлении кода следуйте flake8/pep257, иначе `colcon test` упадёт на этих линтерах. Функциональная проверка — ручная, через симуляцию (`asump_sim`) и RViz.

## Деплой / окружение

- Целевая платформа разработки — нативная Ubuntu (24.04) с ROS 2 Jazzy и Gazebo Harmonic (`ros-jazzy-ros-gz`), Nav2, slam_toolbox, PCL, OpenCV, Flask. GUI (Gazebo, RViz) работает напрямую, без прослоек.
- Визуализация — RViz (входит в `launch_sim.launch.py`); при желании можно поднять Foxglove: `ros2 run foxglove_bridge foxglove_bridge` (порт 8765, пакет `ros-jazzy-foxglove-bridge`).
- `ROS_DOMAIN_ID` берётся из окружения (default 0).
- Боевой запуск на роботе подразумевает наличие `/dev/ttyUSB0` (ESP32 по USB-UART) и лидар; `asump_control` и `ultrasonic_bridge` без железа стартуют с ошибкой порта.

## Безопасность

- Скорости из всех источников проходят через `twist_mux`; у аварийного входа `cmd_vel_safe` (нода `obstacle_safety_gate_node`) наивысший приоритет — не понижайте его в `twist_mux.yaml` и не публикуйте в `cmd_vel` напрямую в обход мультиплексора.
- Веб-телеуправление слушает `0.0.0.0:5000` без авторизации — только для доверенной сети.
