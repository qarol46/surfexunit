import serial
import struct
import time

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(2) 

vx = 0.0
vz = 0.0

aux_motor_on = 1  
# 0: стоп, 1: движение влево (к пину 4), 2: движение вправо (к пину 12)
stepper_cmd = 2  

print("Начало отправки данных. Нажмите Ctrl+C для остановки.")

try:
    while True:
        # Упаковываем данные:
        # '<' : little-endian (порядок байтов, совместимый с ESP32)
        # 'f' : float (4 байта) для vx
        # 'f' : float (4 байта) для vz
        # 'B' : unsigned char (1 байт) для состояния доп. мотора
        # 'B' : unsigned char (1 байт) для состояния шагового двигателя
        # Итого payload = 4 + 4 + 1 + 1 = 10 байт
        payload = struct.pack('<ffBB', vx, vz, aux_motor_on, stepper_cmd)

        cksum = 0
        for b in payload:
            cksum ^= b

        # Собираем кадр: Старт (0xAA) + Данные (10 байт) + Чексумма + Конец (0x55)
        frame = b'\xAA' + payload + bytes([cksum]) + b'\x55'

        s.write(frame)
    
        print(f'vx={vx:4.1f}, vz={vz:4.1f}, aux={aux_motor_on}, stepper={stepper_cmd} | Frame: {frame.hex()}')
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nОстановка отправки данных.")
    # Отправляем команду на полную остановку всех моторов (включая шаговик)
    stop_payload = struct.pack('<ffBB', 0.0, 0.0, 0, 0)
    stop_cksum = 0
    for b in stop_payload:
        stop_cksum ^= b
    stop_frame = b'\xAA' + stop_payload + bytes([stop_cksum]) + b'\x55'
    s.write(stop_frame)
    s.close()