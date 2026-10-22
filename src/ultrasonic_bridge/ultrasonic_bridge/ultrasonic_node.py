import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import struct

class UltrasonicSerialNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # 1. Параметры подключения
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('debug', False)
        
        # 2. Параметр с именами линков для каждого из 8 датчиков
        default_frame_ids = [
            'face_right_us_sensor_link',
            'right1_us_sensor_link',
            'right2_us_sensor_link',
            'back_right_us_sensor_link',
            'face_left_us_sensor_link',
            'left1_us_sensor_link',
            'left2_us_sensor_link',
            'back_left_us_sensor_link',
        ]
        self.declare_parameter('frame_ids', default_frame_ids)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.debug = self.get_parameter('debug').value
        
        # 3. Получаем и валидируем массив линков
        frame_ids_param = self.get_parameter('frame_ids').value
        
        if len(frame_ids_param) != 8:
            self.get_logger().warn(
                f"Ожидается 8 frame_id, получено {len(frame_ids_param)}. "
                f"Дополняем/обрезаем до 8."
            )
            if len(frame_ids_param) < 8:
                frame_ids_param += default_frame_ids[len(frame_ids_param):]
            else:
                frame_ids_param = frame_ids_param[:8]
        
        self.frame_ids = list(frame_ids_param)
        
        # 4. Создаём 8 издателей (Publisher)
        # ⚠️ ИСПРАВЛЕНО: используем sonar_publishers вместо publishers,
        # чтобы не конфликтовать со встроенным свойством Node.publishers
        self.sonar_publishers = []
        for i in range(8):
            topic_name = f'/ultrasonic_{i}'
            pub = self.create_publisher(Range, topic_name, 10)
            self.sonar_publishers.append(pub)
            
        self.get_logger().info("Создано 8 publishers для ультразвуковых датчиков:")
        for i, fid in enumerate(self.frame_ids):
            self.get_logger().info(f"  Датчик {i}: topic=/ultrasonic_{i}, frame_id={fid}")
        
        # 5. Инициализация Serial порта
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Serial port {port} открыт на скорости {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Не удалось открыть serial port {port}: {e}")
            self.ser = None
            
        self.buffer = bytearray()
        
        # 6. Таймер опроса Serial (20мс = 50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if not self.ser or not self.ser.is_open:
            return
            
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                data = self.ser.read(waiting)
                self.buffer.extend(data)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            self.ser.close()
            self.ser = None
            return
            
        self.parse_buffer()

    def parse_buffer(self):
        """
        Парсер бинарного кадра.
        Формат: 0xAA + [32 байта payload] + [1 байт XOR checksum] + 0x55
        """
        while True:
            try:
                start_idx = self.buffer.index(0xAA)
            except ValueError:
                self.buffer.clear()
                break
                
            if start_idx > 0:
                del self.buffer[:start_idx]
                
            if len(self.buffer) < 35:
                break
                
            if self.buffer[34] != 0x55:
                del self.buffer[0]
                continue
                
            payload = self.buffer[1:33]
            recv_checksum = self.buffer[33]
            
            calc_checksum = 0
            for b in payload:
                calc_checksum ^= b
                
            if calc_checksum == recv_checksum:
                distances = struct.unpack('<8f', bytes(payload))
                self.process_distances(distances)
                del self.buffer[:35]
            else:
                del self.buffer[0]
                continue

    def process_distances(self, distances):
        """Формирует и публикует сообщения Range для каждого из 8 датчиков."""
        now = self.get_clock().now().to_msg()
        
        for i in range(8):
            dist_cm = distances[i]
            
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_ids[i]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26  # ~15 градусов
            msg.min_range = 0.02      # 2 см
            msg.max_range = 4.0       # 4 метра
            
            if 0.0 < dist_cm < 400.0:
                msg.range = dist_cm / 100.0
            else:
                msg.range = float('inf')
                
            # ⚠️ ИСПРАВЛЕНО: используем sonar_publishers
            self.sonar_publishers[i].publish(msg)
            
        if self.debug:
            dist_str = ", ".join(
                [f"{self.frame_ids[i]}={d:.1f}см" if d > 0 else f"{self.frame_ids[i]}=---" 
                 for i, d in enumerate(distances)]
            )
            self.get_logger().info(f"Distances: [{dist_str}]")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()