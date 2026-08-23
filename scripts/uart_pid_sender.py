import serial
import struct
import time
import matplotlib.pyplot as plt
from collections import deque

# --- НАСТРОЙКИ ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

# Тестовые команды (линейная и угловая скорость)
VX_CMD = 0.2  # м/с
VZ_CMD = 0.0  # рад/с

# Время проведения теста (секунды)
TEST_DURATION_SEC = 2.0 

# Параметры робота (должны совпадать с теми, что в скетче ESP32!)
WHEEL_RADIUS = 0.19  # метры
WHEEL_BASE = 0.49    # метры

# Расчет целевых скоростей колес в рад/с для графика
V_LEFT_CMD_RAD = (VX_CMD - (VZ_CMD * WHEEL_BASE / 2.0)) / WHEEL_RADIUS
V_RIGHT_CMD_RAD = (VX_CMD + (VZ_CMD * WHEEL_BASE / 2.0)) / WHEEL_RADIUS

def create_frame(vx, vz):
    """Создает байтовый кадр для отправки на ESP32"""
    payload = struct.pack('<ff', vx, vz)
    cksum = 0
    for b in payload:
        cksum ^= b
    return b'\xAA' + payload + bytes([cksum]) + b'\x55'

def main():
    print(f"Открытие порта {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        time.sleep(2)  # Даем время на перезагрузку ESP32 после открытия порта
        ser.reset_input_buffer()
        print("Порт открыт успешно.")
    except Exception as e:
        print(f"Ошибка открытия порта: {e}")
        return

    # Буферы для хранения данных графика
    timestamps = deque()
    vel_left_actual = deque()
    vel_right_actual = deque()

    # Формируем кадр команды
    cmd_frame = create_frame(VX_CMD, VZ_CMD)
    stop_frame = create_frame(0.0, 0.0)
    
    print(f"Начало теста на {TEST_DURATION_SEC} секунд...")
    print(f"Команда: vx={VX_CMD}, vz={VZ_CMD}")
    
    start_time = time.time()
    last_send_time = 0
    serial_buffer = bytearray()

    try:
        while (time.time() - start_time) < TEST_DURATION_SEC:
            now = time.time()
            elapsed = now - start_time

            # Отправляем команду каждые 50 мс, чтобы не срабатывал watchdog
            if now - last_send_time > 0.05:
                ser.write(cmd_frame)
                last_send_time = now

            # Читаем данные из порта
            if ser.in_waiting > 0:
                serial_buffer.extend(ser.read(ser.in_waiting))
                
                # Парсинг кадров
                while True:
                    start_idx = serial_buffer.find(b'\xBB')
                    if start_idx == -1:
                        serial_buffer.clear()
                        break
                    
                    # Длина кадра: 0xBB(1) + payload(16) + cksum(1) + 0x55(1) = 19 байт
                    if len(serial_buffer) < start_idx + 19:
                        break # Ждем оставшиеся байты
                    
                    frame = serial_buffer[start_idx : start_idx + 19]
                    
                    if frame[-1] == 0x55:
                        payload = frame[1:17]
                        cksum_recv = frame[17]
                        
                        # Проверка чексуммы
                        cksum_calc = 0
                        for b in payload:
                            cksum_calc ^= b
                            
                        if cksum_calc == cksum_recv:
                            # Распаковываем: 2 x int32 (позиции), 2 x float (скорости в рад/с)
                            pos_L, pos_R, vel_L, vel_R = struct.unpack('<iiff', payload)
                            
                            timestamps.append(elapsed)
                            vel_left_actual.append(vel_L)
                            vel_right_actual.append(vel_R)
                    
                    # Сдвигаем буфер, чтобы искать следующий кадр
                    serial_buffer = serial_buffer[start_idx + 1:]

    except KeyboardInterrupt:
        print("\nТест прерван пользователем.")
    finally:
        # Обязательно отправляем команду остановки!
        print("Отправка команды остановки...")
        for _ in range(5): # Отправляем несколько раз для надежности
            ser.write(stop_frame)
            time.sleep(0.05)
        ser.close()
        print("Порт закрыт.")

    # --- ПОСТРОЕНИЕ ГРАФИКА ---
    if not timestamps:
        print("Не получено данных от энкодеров. Проверьте подключение и скетч на ESP32.")
        return

    plt.figure(figsize=(10, 6))
    
    # Целевые скорости (пунктирные линии)
    plt.axhline(y=V_LEFT_CMD_RAD, color='r', linestyle='--', label=f'Цель Left (рад/с): {V_LEFT_CMD_RAD:.2f}')
    plt.axhline(y=V_RIGHT_CMD_RAD, color='b', linestyle='--', label=f'Цель Right (рад/с): {V_RIGHT_CMD_RAD:.2f}')
    
    # Фактические скорости (сплошные линии)
    plt.plot(timestamps, vel_left_actual, color='red', alpha=0.7, label='Факт Left (рад/с)')
    plt.plot(timestamps, vel_right_actual, color='blue', alpha=0.7, label='Факт Right (рад/с)')

    plt.title('Реакция ПИД-регулятора')
    plt.xlabel('Время (сек)')
    plt.ylabel('Угловая скорость колеса (рад/с)')
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    
    # Ограничиваем ось X длительностью теста
    plt.xlim(0, TEST_DURATION_SEC)
    
    print("Отображение графика... Закройте окно графика для завершения программы.")
    plt.show()

if __name__ == '__main__':
    main()