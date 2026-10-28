#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool, Int8
import serial
import struct
import threading
import math

class VelocityForwarderNode(Node):
    def __init__(self):
        super().__init__('velocity_forwarder')

        # Параметры последовательного порта
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('aux_motor_topic', '/aux_motor_cmd')
        self.declare_parameter('stepper_topic', '/stepper_cmd')  # НОВЫЙ ТОПИК

        # Параметры кинематики
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.5)

        # Четыре джойнта
        self.declare_parameter('wheel_left_1_name',  'fl_wheel_joint')
        self.declare_parameter('wheel_left_2_name',  'rl_wheel_joint')
        self.declare_parameter('wheel_right_1_name', 'fr_wheel_joint')
        self.declare_parameter('wheel_right_2_name', 'rr_wheel_joint')

        self.declare_parameter('joint_state_rate', 50.0)
        self.declare_parameter('aux_motor_state_rate', 5.0)

        self.port_name = self.get_parameter('serial_port').value
        self.baudrate  = self.get_parameter('baudrate').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.aux_motor_topic = self.get_parameter('aux_motor_topic').value
        self.stepper_topic = self.get_parameter('stepper_topic').value

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base   = self.get_parameter('wheel_base').value

        self.wheel_left_1_name  = self.get_parameter('wheel_left_1_name').value
        self.wheel_left_2_name  = self.get_parameter('wheel_left_2_name').value
        self.wheel_right_1_name = self.get_parameter('wheel_right_1_name').value
        self.wheel_right_2_name = self.get_parameter('wheel_right_2_name').value

        self.joint_state_rate = self.get_parameter('joint_state_rate').value
        self.aux_motor_state_rate = self.get_parameter('aux_motor_state_rate').value

        self.ser_lock = threading.Lock()
        self.ser = None
        self._open_serial()

        self.state_lock = threading.Lock()
        self.last_linear_x  = 0.0
        self.last_angular_z = 0.0

        # Состояния моторов: 0 = ВЫКЛ/СТОП, 1 = ВКЛ/ДВИЖЕНИЕ
        self.aux_motor_state = 0
        self.stepper_state = 0  # 0: стоп, 1: влево, 2: вправо

        self.angle_left_1  = 0.0
        self.angle_left_2  = 0.0
        self.angle_right_1 = 0.0
        self.angle_right_2 = 0.0

        self.vel_left  = 0.0
        self.vel_right = 0.0

        self.last_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self._velocity_callback, 10)

        self.aux_motor_sub = self.create_subscription(
            Bool, self.aux_motor_topic, self._aux_motor_callback, 10)

        # ПОДПИСКА НА ТОПИК ШАГОВОГО ДВИГАТЕЛЯ
        self.stepper_sub = self.create_subscription(
            Int8, self.stepper_topic, self._stepper_callback, 10)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Таймер для публикации JointState
        period = 1.0 / self.joint_state_rate
        self.joint_timer = self.create_timer(period, self._joint_state_timer_cb)

        # Таймер для периодической отправки состояния моторов
        aux_period = 1.0 / self.aux_motor_state_rate
        self.aux_motor_timer = self.create_timer(aux_period, self._aux_motor_timer_cb)

        self.get_logger().info(
            f'Node started | CMD: {self.cmd_topic} | Aux: {self.aux_motor_topic} | '
            f'Stepper: {self.stepper_topic} | Port: {self.port_name} | '
            f'Wheel R={self.wheel_radius} m, Base={self.wheel_base} m'
        )

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

    def _send_frame(self, linear_x: float, angular_z: float, aux_state: int, stepper_state: int):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn('Serial disconnected. Reconnecting...')
            if not self._open_serial():
                return False
        try:
            # УПАКОВКА 10 БАЙТ: 2 float (8 байт) + 2 uint8 (2 байта) = 10 байт
            payload = struct.pack('<ffBB', linear_x, angular_z, aux_state, stepper_state)
            cksum = 0
            for b in payload:
                cksum ^= b
            frame = b'\xAA' + payload + bytes([cksum]) + b'\x55'
            with self.ser_lock:
                self.ser.write(frame)
                self.ser.flush()
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f'Write error: {e}. Port will reopen on next message.')
            try:
                with self.ser_lock:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            return False

    def _aux_motor_callback(self, msg: Bool):
        """Обработчик топика вспомогательного мотора"""
        self.aux_motor_state = 1 if msg.data else 0
        state_str = "ON" if msg.data else "OFF"
        self.get_logger().info(f"Auxiliary motor turned {state_str} (state={self.aux_motor_state})")
        with self.state_lock:
            self._send_frame(self.last_linear_x, self.last_angular_z, 
                           self.aux_motor_state, self.stepper_state)

    def _stepper_callback(self, msg: Int8):
        """Обработчик топика шагового двигателя"""
        # Ограничиваем значения: 0 (стоп), 1 (влево), 2 (вправо)
        new_state = max(0, min(2, msg.data))
        if new_state != self.stepper_state:
            self.stepper_state = new_state
            state_names = {0: "STOP", 1: "LEFT", 2: "RIGHT"}
            self.get_logger().info(f"Stepper command: {state_names.get(new_state, 'UNKNOWN')}")
            with self.state_lock:
                self._send_frame(self.last_linear_x, self.last_angular_z,
                               self.aux_motor_state, self.stepper_state)

    def _aux_motor_timer_cb(self):
        """Периодическая отправка состояния всех моторов"""
        with self.state_lock:
            self._send_frame(self.last_linear_x, self.last_angular_z,
                           self.aux_motor_state, self.stepper_state)

    def _velocity_callback(self, msg: Twist):
        with self.state_lock:
            self.last_linear_x  = float(msg.linear.x)
            self.last_angular_z = float(msg.angular.z)
            self._send_frame(msg.linear.x, msg.angular.z,
                           self.aux_motor_state, self.stepper_state)

    def _joint_state_timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.joint_state_rate

        with self.state_lock:
            v = self.last_linear_x
            w = self.last_angular_z

            v_left  = v - w * self.wheel_base / 2.0
            v_right = v + w * self.wheel_base / 2.0

            if self.wheel_radius > 0.0:
                self.vel_left  = v_left  / self.wheel_radius
                self.vel_right = v_right / self.wheel_radius
            else:
                self.vel_left  = 0.0
                self.vel_right = 0.0

            self.angle_left_1  += self.vel_left  * dt
            self.angle_left_2  += self.vel_left  * dt
            self.angle_right_1 += self.vel_right * dt
            self.angle_right_2 += self.vel_right * dt

            self.angle_left_1  = math.atan2(math.sin(self.angle_left_1),  math.cos(self.angle_left_1))
            self.angle_left_2  = math.atan2(math.sin(self.angle_left_2),  math.cos(self.angle_left_2))
            self.angle_right_1 = math.atan2(math.sin(self.angle_right_1), math.cos(self.angle_right_1))
            self.angle_right_2 = math.atan2(math.sin(self.angle_right_2), math.cos(self.angle_right_2))

        js = JointState()
        js.header = Header()
        js.header.stamp = now.to_msg()
        js.header.frame_id = ''
        js.name = [
            self.wheel_left_1_name,
            self.wheel_left_2_name,
            self.wheel_right_1_name,
            self.wheel_right_2_name
        ]
        js.position = [
            self.angle_left_1,
            self.angle_left_2,
            self.angle_right_1,
            self.angle_right_2
        ]
        js.velocity = [
            self.vel_left,
            self.vel_left,
            self.vel_right,
            self.vel_right
        ]
        js.effort = []
        self.joint_pub.publish(js)

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