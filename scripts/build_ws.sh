#!/bin/bash
# Сборка colcon-workspace.
#
# Запуск из контейнера (или из любой Linux-среды с ROS):
#   ./scripts/build_ws.sh                      # полная сборка
#   ./scripts/build_ws.sh asump_nav asump_sim  # точечная сборка пакетов
#   ./scripts/build_ws.sh --clean              # удалить build/ install/ log/ и собрать с нуля
#
# Запуск с хоста (macOS): скрипт сам перенаправит сборку
# в запущенный контейнер surfexunit_sim_jazzy.

CONTAINER_NAME="surfexunit_sim_jazzy"

if ! command -v colcon >/dev/null 2>&1; then
    if command -v docker >/dev/null 2>&1 \
        && [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" = "true" ]; then
        echo "colcon на хосте не найден — собираю внутри контейнера $CONTAINER_NAME..."
        exec docker exec "$CONTAINER_NAME" bash -c \
            "source /opt/ros/jazzy/setup.bash && cd /workspace && ./scripts/build_ws.sh $*"
    fi
    echo "Ошибка: colcon не найден." >&2
    echo "Скрипт нужно запускать внутри контейнера (./scripts/run.sh)" >&2
    echo "либо запустить контейнер $CONTAINER_NAME на хосте." >&2
    exit 1
fi

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

CLEAN=0
if [ "$1" = "--clean" ]; then
    CLEAN=1
    shift
fi

if [ "$CLEAN" = "1" ]; then
    echo "Удаляю build/ install/ log/ ..."
    rm -rf build install log
fi

if [ "$#" -gt 0 ]; then
    colcon build --symlink-install --packages-select "$@"
else
    colcon build --symlink-install
fi

echo ""
echo "Сборка завершена. Примените окружение:"
echo "  source $WS_DIR/install/setup.bash"
