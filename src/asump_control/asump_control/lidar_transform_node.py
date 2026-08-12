#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import copy

class LidarRotate180Node(Node):
    def __init__(self):
        super().__init__('lidar_rotate_180_node')

        # Объявляем параметры, чтобы топики можно было менять при запуске
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_rotated')

        # Читаем параметры
        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Создаем подписчик и издатель
        self.subscription = self.create_subscription(
            LaserScan,
            in_topic,
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, out_topic, 10)

        self.get_logger().info(f'Нода запущена!')
        self.get_logger().info(f'Подписка на: "{in_topic}"')
        self.get_logger().info(f'Публикация в: "{out_topic}"')

    def scan_callback(self, msg: LaserScan):
        # Глубокое копирование, чтобы не модифицировать оригинальное сообщение
        rotated_scan = copy.deepcopy(msg)

        # Поворот на 180 градусов (pi радиан) вокруг оси Z
        # Физический порядок лучей не меняется, меняется только их угол
        rotated_scan.angle_min = msg.angle_min + math.pi
        rotated_scan.angle_max = msg.angle_max + math.pi
        
        # angle_increment, time_increment и сами массивы ranges остаются без изменений

        self.publisher.publish(rotated_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRotate180Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()