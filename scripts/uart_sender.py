import serial, struct, time

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
vx, vz = 4.0, 0.0  # Тестовые скорости

payload = struct.pack('<ff', vx, vz)
cksum = 0

for b in payload: cksum ^= b

frame = b'\xAA' + payload + bytes([cksum]) + b'\x55'

print(f'Sending frame: {frame.hex()}')

while True:
    s.write(frame)
    time.sleep(0.1)