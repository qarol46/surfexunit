#!/bin/bash

CONTAINER_NAME="surfexunit_container"

# Проверка, запущен ли контейнер
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Контейнер '$CONTAINER_NAME' не запущен. Сначала выполните './run.sh'"
    exit 1
fi

# Определение доступного эмулятора терминала
if command -v gnome-terminal >/dev/null 2>&1; then
    TERMINAL_CMD="gnome-terminal --"
elif command -v konsole >/dev/null 2>&1; then
    TERMINAL_CMD="konsole -e"
elif command -v xterm >/dev/null 2>&1; then
    TERMINAL_CMD="xterm -e"
else
    echo "Не удалось найти подходящий эмулятор терминала. Выполняем в текущем окне:"
    docker exec -it "$CONTAINER_NAME" /bin/bash
    exit 0
fi

echo "Открытие нового окна терминала в контейнере '$CONTAINER_NAME'..."

# Запуск нового окна с интерактивной сессией bash
$TERMINAL_CMD docker exec -it "$CONTAINER_NAME" /bin/bash &

echo "Новое окно открыто. Вы можете запускать этот скрипт несколько раз для многозадачности."