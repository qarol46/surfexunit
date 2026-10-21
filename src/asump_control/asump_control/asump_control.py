#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool
import serial
import struct
import threading
import math

class VelocityForwarderNode(Node):
    def __init__(self):
        super().__init__('velocity_forwarder')
        
        self.declare_parameter('serial_port', '/dev/ttyESP1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('wheel_left_name', 'left_wheel_joint')
        self.declare_parameter('wheel_right_name', 'right_wheel_joint')
        self.declare_parameter('joint_state_rate', 50.0)
        
        self.declare_parameter('aux_motor_service_name', 'set_aux_motor')

        self.port_name = self.get_parameter('serial_port').value
        self.baudrate  = self.get_parameter('baudrate').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base   = self.get_parameter('wheel_base').value
        self.wheel_left_name  = self.get_parameter('wheel_left_name').value
        self.wheel_right_name = self.get_parameter('wheel_right_name').value
        self.joint_state_rate = self.get_parameter('joint_state_rate').value
        self.aux_motor_service_name = self.get_parameter('aux_motor_service_name').value

        self.ser_lock = threading.Lock()
        self.ser = None
        self._open_serial()
        
        self.state_lock = threading.Lock()
        self.last_linear_x  = 0.0
        self.last_angular_z = 0.0
        
        self.aux_motor_state = 0 
        
        self.angle_left  = 0.0
        self.angle_right = 0.0
        self.vel_left  = 0.0
        self.vel_right = 0.0
        self.last_time = self.get_clock().now()
        
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self._velocity_callback, 10)
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.aux_motor_srv = self.create_service(
            SetBool, self.aux_motor_service_name, self._aux_motor_callback)
            
        period = 1.0 / self.joint_state_rate
        self.joint_timer = self.create_timer(period, self._joint_state_timer_cb)
        
        self.get_logger().info(
            f'Node started | CMD: {self.cmd_topic} | Port: {self.port_name} | '
            f'Wheel R={self.wheel_radius} m, Base={self.wheel_base} m, '
            f'Joints: [{self.wheel_left_name}, {self.wheel_right_name}] @ {self.joint_state_rate} Hz | '
            f'Aux Motor Service: {self.aux_motor_service_name}'
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

    def _aux_motor_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.aux_motor_state = 1 if request.data else 0
        state_str = "ON" if request.data else "OFF"
        response.success = True
        response.message = f"Auxiliary motor turned {state_str} (state={self.aux_motor_state})"
        self.get_logger().info(response.message)
        return response

    def _velocity_callback(self, msg: Twist):
        with self.state_lock:
            self.last_linear_x  = float(msg.linear.x)
            self.last_angular_z = float(msg.angular.z)
            
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn('Serial disconnected. Reconnecting...')
            if not self._open_serial():
                return
                
        try:
            with self.state_lock:
                payload = struct.pack('<ffB', msg.linear.x, msg.angular.z, self.aux_motor_state)
            
            cksum = 0
            for b in payload:
                cksum ^= b
                
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
                
            self.angle_left  += self.vel_left  * dt
            self.angle_right += self.vel_right * dt
            
            # Нормализация угла в диапазон [-pi, pi]
            self.angle_left  = math.atan2(math.sin(self.angle_left),  math.cos(self.angle_left))
            self.angle_right = math.atan2(math.sin(self.angle_right), math.cos(self.angle_right))
            
        js = JointState()
        js.header = Header()
        js.header.stamp = now.to_msg()
        js.header.frame_id = ''
        js.name     = [self.wheel_left_name, self.wheel_right_name]
        js.position = [self.angle_left, self.angle_right]
        js.velocity = [self.vel_left,  self.vel_right]
        js.effort   = []
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