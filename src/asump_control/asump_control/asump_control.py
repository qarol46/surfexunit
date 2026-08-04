#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import threading

class VelocityForwarderNode(Node):
    def __init__(self):
        super().__init__('velocity_forwarder')
        
        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        
        self.port_name = self.get_parameter('serial_port').value
        self.baudrate  = self.get_parameter('baudrate').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        
        self.ser_lock = threading.Lock()
        self.ser = None
        self._open_serial()
        
        # Подписка на скорости
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self._velocity_callback, 10)
        
        self.get_logger().info(
            f'Node started | CMD: {self.cmd_topic} | Port: {self.port_name}')

    def _open_serial(self) -> bool:
        try:
            with self.ser_lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05,
                    write_timeout=0.05
                )
                self.get_logger().info(f'Serial {self.port_name} opened at {self.baudrate}')
                return True
        except Exception as e:
            self.get_logger().error(f'Serial open failed: {e}')
            self.ser = None
            return False

    def _velocity_callback(self, msg: Twist):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn('Serial disconnected. Reconnecting...')
            if not self._open_serial():
                return
        
        try:
            # Упаковываем linear.x и angular.z в 2 float (8 байт)
            payload = struct.pack('<ff', msg.linear.x, msg.angular.z)
            
            # Считаем XOR чексумму
            cksum = 0
            for b in payload:
                cksum ^= b
                
            # Формируем кадр: AA + payload + cksum + 55
            frame = b'\xAA' + payload + bytes([cksum]) + b'\x55'
            
            with self.ser_lock:
                self.ser.write(frame)
                self.ser.flush()
                
        except serial.SerialException as e:
            self.get_logger().warn(f'Write error: {e}. Port will reopen on next message.')
            try:
                with self.ser_lock:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            with self.ser_lock:
                self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityForwarderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()