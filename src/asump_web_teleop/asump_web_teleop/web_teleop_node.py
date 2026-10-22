import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, Int8
from flask import Flask, render_template_string, jsonify, request, Response
from flask_cors import CORS
import threading
import socket
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop_node')
        
        # Публикуем в /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Публикуем в топик вспомогательного мотора
        self.motor_publisher = self.create_publisher(Bool, '/aux_motor_cmd', 10)
        # Публикуем в топик шагового двигателя
        self.stepper_publisher = self.create_publisher(Int8, '/stepper_cmd', 10)
        
        # Подписываемся на /scan
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan_rotated', self.scan_callback, 10
        )
        # Подписываемся на камеру
        self.camera_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.camera_callback, 10
        )
        
        self.motor_state = False
        self.stepper_state = 0  # 0: стоп, 1: влево, 2: вправо
        
        # Текущие значения скорости
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Настраиваемые параметры скоростей
        self.max_linear = 1.0
        self.max_angular = 1.0
        self.angular_deadzone = 0.0
        
        # Данные лидара
        self.scan_data = {
            'ranges': [],
            'angle_min': 0.0,
            'angle_max': 0.0,
            'angle_increment': 0.0,
            'range_min': 0.0,
            'range_max': 0.0
        }
        
        # Данные камеры
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.cv_bridge = CvBridge()
        
        # Flask
        self.app = Flask(__name__)
        CORS(self.app)
        
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        @self.app.route('/cmd_vel', methods=['POST'])
        def cmd_vel():
            try:
                data = request.json
                if data is None:
                    return jsonify({'status': 'error', 'message': 'No JSON data'}), 400
                
                raw_linear = float(data.get('linear', 0.0) or 0.0)
                raw_angular = float(data.get('angular', 0.0) or 0.0)
                
                if abs(raw_angular) < self.angular_deadzone:
                    raw_angular = 0.0
                
                linear = max(-self.max_linear, min(self.max_linear, raw_linear))
                angular = max(-self.max_angular, min(self.max_angular, raw_angular))
                
                self.current_linear = linear
                self.current_angular = angular
                
                return jsonify({'status': 'ok'})
            except Exception as e:
                self.get_logger().error(f'Error in /cmd_vel: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @self.app.route('/motor', methods=['POST'])
        def motor():
            try:
                data = request.json
                if data is None:
                    return jsonify({'status': 'error', 'message': 'No JSON data'}), 400
                
                enable = bool(data.get('enable', False))
                msg = Bool()
                msg.data = enable
                self.motor_publisher.publish(msg)
                self.motor_state = enable
                
                self.get_logger().info(f'Motor {"enabled" if enable else "disabled"}')
                return jsonify({
                    'status': 'ok',
                    'enabled': enable
                })
            except Exception as e:
                self.get_logger().error(f'Error in /motor: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @self.app.route('/motor/status')
        def motor_status():
            return jsonify({'enabled': self.motor_state})
        
        # НОВЫЙ ENDPOINT ДЛЯ ШАГОВОГО ДВИГАТЕЛЯ
        @self.app.route('/stepper', methods=['POST'])
        def stepper():
            try:
                data = request.json
                if data is None:
                    return jsonify({'status': 'error', 'message': 'No JSON data'}), 400
                
                # Ожидаем: command = 0 (стоп), 1 (влево), 2 (вправо)
                command = int(data.get('command', 0))
                command = max(0, min(2, command))  # Ограничение диапазона
                
                msg = Int8()
                msg.data = command
                self.stepper_publisher.publish(msg)
                self.stepper_state = command
                
                state_names = {0: "STOP", 1: "LEFT", 2: "RIGHT"}
                self.get_logger().info(f'Stepper command: {state_names.get(command, "UNKNOWN")}')
                
                return jsonify({
                    'status': 'ok',
                    'command': command
                })
            except Exception as e:
                self.get_logger().error(f'Error in /stepper: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @self.app.route('/stepper/status')
        def stepper_status():
            return jsonify({'command': self.stepper_state})
        
        @self.app.route('/config', methods=['GET'])
        def get_config():
            return jsonify({
                'max_linear': self.max_linear,
                'max_angular': self.max_angular,
                'angular_deadzone': self.angular_deadzone
            })
        
        @self.app.route('/config', methods=['POST'])
        def set_config():
            try:
                data = request.json
                if data is None:
                    return jsonify({'status': 'error'}), 400
                
                if 'max_linear' in data:
                    self.max_linear = max(0.1, min(3.0, float(data['max_linear'])))
                if 'max_angular' in data:
                    self.max_angular = max(0.1, min(3.0, float(data['max_angular'])))
                if 'angular_deadzone' in data:
                    self.angular_deadzone = max(0.0, min(1.0, float(data['angular_deadzone'])))
                
                return jsonify({'status': 'ok'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @self.app.route('/scan')
        def scan():
            return jsonify(self.scan_data)
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame',
                          headers={
                              'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
                              'Pragma': 'no-cache',
                              'Expires': '0',
                              'Connection': 'close'
                          })
        
        self.ip_address = self.get_ip_address()
        self.port = 5000
        
        self.get_logger().info(f'Web Teleop Node запущена!')
        self.get_logger().info(f'Откройте в браузере: http://{self.ip_address}:{self.port}')
        
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
    
    def scan_callback(self, msg):
        try:
            ranges = []
            for r in msg.ranges:
                if math.isfinite(r) and r >= msg.range_min and r <= msg.range_max:
                    ranges.append(float(r))
                else:
                    ranges.append(None)
            
            self.scan_data = {
                'ranges': ranges,
                'angle_min': float(msg.angle_min),
                'angle_max': float(msg.angle_max),
                'angle_increment': float(msg.angle_increment),
                'range_min': float(msg.range_min),
                'range_max': float(msg.range_max)
            }
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {e}')
    
    def camera_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error in camera_callback: {e}')
    
    def generate_frames(self):
        """Генерация MJPEG потока с ограничением FPS для мобильных устройств"""
        last_frame_time = 0
        target_fps = 15  # Ограничиваем до 15 FPS для стабильности на телефонах
        frame_interval = 1.0 / target_fps
        
        while True:
            current_time = time.monotonic()
            
            # Ограничиваем частоту кадров
            if current_time - last_frame_time < frame_interval:
                time.sleep(0.01)
                continue
            
            with self.frame_lock:
                if self.latest_frame is None:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                else:
                    frame = self.latest_frame.copy()
            
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ret:
                continue
            
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            last_frame_time = current_time
    
    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return '127.0.0.1'
    
    def run_flask(self):
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        self.app.run(host='0.0.0.0', port=self.port, debug=False, use_reloader=False)
    
    def publish_cmd_vel(self):
        try:
            msg = Twist()
            msg.linear.x = float(self.current_linear)
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(self.current_angular)
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing cmd_vel: {e}')

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Robot Control Panel</title>
    <script src="https://cdn.jsdelivr.net/npm/nipplejs@0.10.1/dist/nipplejs.min.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Courier New', monospace;
            background: #0a0a0a;
            color: #ffffff;
            padding: 10px;
            height: 100vh;
            overflow: hidden;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            height: 100%;
            display: flex;
            flex-direction: column;
        }
        .header {
            border-bottom: 2px solid #ff6600;
            padding-bottom: 10px;
            margin-bottom: 15px;
            flex-shrink: 0;
        }
        h1 {
            font-size: 20px;
            font-weight: bold;
            color: #ff6600;
            text-transform: uppercase;
            letter-spacing: 2px;
        }
        .subtitle { font-size: 11px; color: #888; margin-top: 3px; }
        .main-content {
            flex: 1;
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: 1fr 1fr;
            gap: 15px;
            min-height: 0;
        }
        .section {
            background: #1a1a1a;
            border: 1px solid #333;
            padding: 15px;
            display: flex;
            flex-direction: column;
            min-height: 0;
            overflow: hidden;
        }
        .section-title {
            font-size: 12px;
            color: #ff6600;
            text-transform: uppercase;
            margin-bottom: 10px;
            letter-spacing: 1px;
            flex-shrink: 0;
        }
        .section-content {
            flex: 1;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            min-height: 0;
        }
        #lidarCanvas {
            max-width: 100%;
            max-height: 100%;
            background: #0a0a0a;
            border: 2px solid #333;
        }
        .lidar-info {
            display: flex;
            justify-content: space-around;
            margin-top: 10px;
            font-size: 11px;
            width: 100%;
            flex-shrink: 0;
        }
        .lidar-stat { text-align: center; }
        .lidar-stat-value { color: #ff6600; font-weight: bold; font-size: 14px; }
        .lidar-stat-label { color: #888; margin-top: 3px; }
        #videoFeed {
            max-width: 100%;
            max-height: 100%;
            background: #0a0a0a;
            border: 2px solid #333;
        }
        #joystick {
            width: 100%;
            max-width: 200px;
            height: 200px;
            background: #0a0a0a;
            border: 2px solid #333;
            position: relative;
        }
        .values {
            background: #0a0a0a;
            border: 1px solid #333;
            padding: 10px;
            margin-top: 10px;
            width: 100%;
            max-width: 300px;
        }
        .value-row {
            display: flex;
            justify-content: space-between;
            padding: 3px 0;
            font-size: 12px;
        }
        .value-label { color: #888; }
        .value-number { color: #ff6600; font-weight: bold; }
        
        /* Tool control */
        .tool-panel {
            width: 100%;
            max-width: 320px;
        }
        .motor-control {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 12px;
            background: #0a0a0a;
            border: 1px solid #333;
            margin-bottom: 12px;
        }
        .motor-status { display: flex; align-items: center; gap: 10px; }
        .status-indicator {
            width: 12px; height: 12px;
            border-radius: 50%;
            background: #333;
            border: 2px solid #555;
        }
        .status-indicator.active { background: #ff6600; border-color: #ff8533; }
        .status-text { font-size: 14px; color: #888; }
        .motor-btn {
            background: #ff6600;
            color: #000;
            border: none;
            padding: 8px 20px;
            font-size: 12px;
            font-weight: bold;
            font-family: 'Courier New', monospace;
            text-transform: uppercase;
            cursor: pointer;
            letter-spacing: 1px;
        }
        .motor-btn:hover { background: #ff8533; }
        .motor-btn:active { background: #cc5200; }
        .motor-btn.active {
            background: #333;
            color: #ff6600;
            border: 1px solid #ff6600;
        }
        
        /* СТЕППЕР КОНТРОЛЬ */
        .stepper-control {
            background: #0a0a0a;
            border: 1px solid #333;
            padding: 12px;
            margin-bottom: 12px;
        }
        .stepper-title {
            font-size: 11px;
            color: #888;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 10px;
            text-align: center;
        }
        .stepper-buttons {
            display: flex;
            gap: 8px;
            justify-content: center;
        }
        .stepper-btn {
            flex: 1;
            background: #333;
            color: #ff6600;
            border: 2px solid #ff6600;
            padding: 12px 8px;
            font-size: 16px;
            font-weight: bold;
            font-family: 'Courier New', monospace;
            cursor: pointer;
            transition: all 0.2s;
        }
        .stepper-btn:hover {
            background: #ff6600;
            color: #000;
        }
        .stepper-btn:active {
            transform: scale(0.95);
        }
        .stepper-btn.active {
            background: #ff6600;
            color: #000;
        }
        .stepper-btn.stop-btn {
            background: #cc0000;
            color: #fff;
            border-color: #ff3333;
        }
        .stepper-btn.stop-btn:hover {
            background: #ff3333;
        }
        .stepper-btn.stop-btn.active {
            background: #ff3333;
        }
        
        /* Ползунки */
        .slider-group {
            background: #0a0a0a;
            border: 1px solid #333;
            padding: 12px;
            margin-bottom: 8px;
        }
        .slider-header {
            display: flex;
            justify-content: space-between;
            margin-bottom: 6px;
            font-size: 11px;
        }
        .slider-label { color: #888; text-transform: uppercase; letter-spacing: 1px; }
        .slider-value { color: #ff6600; font-weight: bold; }
        input[type="range"] {
            -webkit-appearance: none;
            width: 100%;
            height: 4px;
            background: #333;
            outline: none;
            border-radius: 2px;
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 16px;
            height: 16px;
            background: #ff6600;
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid #0a0a0a;
        }
        input[type="range"]::-moz-range-thumb {
            width: 16px;
            height: 16px;
            background: #ff6600;
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid #0a0a0a;
        }
        .stop-btn {
            width: 100%;
            max-width: 300px;
            background: #ff6600;
            color: #000;
            border: none;
            padding: 12px;
            font-size: 14px;
            font-weight: bold;
            font-family: 'Courier New', monospace;
            text-transform: uppercase;
            cursor: pointer;
            letter-spacing: 2px;
            margin-top: 10px;
        }
        .stop-btn:hover { background: #ff8533; }
        .stop-btn:active { background: #cc5200; }
        .status-bar {
            margin-top: 10px;
            padding: 8px;
            background: #0a0a0a;
            border: 1px solid #333;
            font-size: 11px;
            color: #888;
            text-align: center;
            flex-shrink: 0;
        }
        .status-bar.connected { color: #ff6600; border-color: #ff6600; }
        .status-bar.error { color: #ff3333; border-color: #ff3333; }
        
        @media (max-width: 768px) {
            .main-content {
                grid-template-columns: 1fr;
                grid-template-rows: repeat(4, 1fr);
            }
            #joystick { max-width: 150px; height: 150px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Robot Control</h1>
            <div class="subtitle">Teleoperation Interface v1.5</div>
        </div>
        <div class="main-content">
            <div class="section">
                <div class="section-title">Lidar Scan</div>
                <div class="section-content">
                    <canvas id="lidarCanvas" width="300" height="300"></canvas>
                    <div class="lidar-info">
                        <div class="lidar-stat">
                            <div class="lidar-stat-value" id="minDist">--</div>
                            <div class="lidar-stat-label">MIN (m)</div>
                        </div>
                        <div class="lidar-stat">
                            <div class="lidar-stat-value" id="maxDist">--</div>
                            <div class="lidar-stat-label">MAX (m)</div>
                        </div>
                        <div class="lidar-stat">
                            <div class="lidar-stat-value" id="pointsCount">--</div>
                            <div class="lidar-stat-label">POINTS</div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="section">
                <div class="section-title">Camera Feed</div>
                <div class="section-content">
                    <img id="videoFeed" src="/video_feed" alt="Camera Feed">
                </div>
            </div>
            <div class="section">
                <div class="section-title">Movement Control</div>
                <div class="section-content">
                    <div id="joystick"></div>
                    <div class="values">
                        <div class="value-row">
                            <span class="value-label">LINEAR:</span>
                            <span class="value-number"><span id="linear">0.00</span> m/s</span>
                        </div>
                        <div class="value-row">
                            <span class="value-label">ANGULAR:</span>
                            <span class="value-number"><span id="angular">0.00</span> rad/s</span>
                        </div>
                    </div>
                    <button class="stop-btn" onclick="stopRobot()">Emergency Stop</button>
                </div>
            </div>
            <div class="section">
                <div class="section-title">Tool Control</div>
                <div class="section-content">
                    <div class="tool-panel">
                        <div class="motor-control">
                            <div class="motor-status">
                                <div class="status-indicator" id="motorIndicator"></div>
                                <span class="status-text" id="motorText">OFF</span>
                            </div>
                            <button class="motor-btn" id="motorBtn" onclick="toggleMotor()">Enable</button>
                        </div>
                        
                        <!-- УПРАВЛЕНИЕ ШАГОВЫМ ДВИГАТЕЛЕМ -->
                        <div class="stepper-control">
                            <div class="stepper-title">Stepper Motor</div>
                            <div class="stepper-buttons">
                                <button class="stepper-btn" id="stepperLeft" onclick="setStepper(1)">◄</button>
                                <button class="stepper-btn stop-btn" id="stepperStop" onclick="setStepper(0)">■</button>
                                <button class="stepper-btn" id="stepperRight" onclick="setStepper(2)">►</button>
                            </div>
                        </div>
                        
                        <div class="slider-group">
                            <div class="slider-header">
                                <span class="slider-label">Max Linear</span>
                                <span class="slider-value"><span id="maxLinearVal">1.00</span> m/s</span>
                            </div>
                            <input type="range" id="maxLinear" min="0.1" max="3.0" step="0.1" value="1.0">
                        </div>
                        <div class="slider-group">
                            <div class="slider-header">
                                <span class="slider-label">Max Angular</span>
                                <span class="slider-value"><span id="maxAngularVal">1.00</span> rad/s</span>
                            </div>
                            <input type="range" id="maxAngular" min="0.1" max="3.0" step="0.1" value="1.0">
                        </div>
                        <div class="slider-group">
                            <div class="slider-header">
                                <span class="slider-label">Angular Deadzone</span>
                                <span class="slider-value"><span id="deadzoneVal">0.00</span> rad/s</span>
                            </div>
                            <input type="range" id="deadzone" min="0.0" max="1.0" step="0.05" value="0.0">
                        </div>
                    </div>
                </div>
            </div>
        </div>
        <div class="status-bar" id="status">Initializing...</div>
    </div>

    <script>
        let motorEnabled = false;
        let stepperCommand = 0;
        let currentMaxLinear = 1.0;
        let currentMaxAngular = 1.0;
        
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        
        const joystick = nipplejs.create({
            zone: document.getElementById('joystick'),
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#ff6600',
            size: 120
        });
        
        joystick.on('move', function(evt, data) {
            const linear = parseFloat((data.vector.y * currentMaxLinear).toFixed(2));
            const angular = parseFloat((-data.vector.x * currentMaxAngular).toFixed(2));
            updateSpeeds(linear, angular);
            sendCmdVel(linear, angular);
        });
        
        joystick.on('end', function() {
            updateSpeeds(0, 0);
            sendCmdVel(0.0, 0.0);
        });
        
        function updateSpeeds(linear, angular) {
            document.getElementById('linear').textContent = linear.toFixed(2);
            document.getElementById('angular').textContent = angular.toFixed(2);
        }
        
        function sendCmdVel(linear, angular) {
            fetch('/cmd_vel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    linear: Number(linear) || 0.0,
                    angular: Number(angular) || 0.0
                })
            })
            .then(response => response.json())
            .then(() => updateStatus('Connected', 'connected'))
            .catch(() => updateStatus('Connection Error', 'error'));
        }
        
        function stopRobot() {
            updateSpeeds(0, 0);
            sendCmdVel(0.0, 0.0);
            setStepper(0);  // Останавливаем и шаговик
        }
        
        function toggleMotor() {
            const newState = !motorEnabled;
            fetch('/motor', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ enable: newState })
            })
            .then(response => response.json())
            .then(result => {
                if (result.status === 'ok') {
                    motorEnabled = newState;
                    updateMotorUI();
                }
            })
            .catch(error => console.error('Motor error:', error));
        }
        
        function updateMotorUI() {
            const indicator = document.getElementById('motorIndicator');
            const text = document.getElementById('motorText');
            const btn = document.getElementById('motorBtn');
            
            if (motorEnabled) {
                indicator.classList.add('active');
                text.textContent = 'ON';
                text.style.color = '#ff6600';
                btn.textContent = 'Disable';
                btn.classList.add('active');
            } else {
                indicator.classList.remove('active');
                text.textContent = 'OFF';
                text.style.color = '#888';
                btn.textContent = 'Enable';
                btn.classList.remove('active');
            }
        }
        
        // УПРАВЛЕНИЕ ШАГОВЫМ ДВИГАТЕЛЕМ
        function setStepper(command) {
            stepperCommand = command;
            fetch('/stepper', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: command })
            })
            .then(response => response.json())
            .then(result => {
                if (result.status === 'ok') {
                    updateStepperUI();
                }
            })
            .catch(error => console.error('Stepper error:', error));
        }
        
        function updateStepperUI() {
            const leftBtn = document.getElementById('stepperLeft');
            const stopBtn = document.getElementById('stepperStop');
            const rightBtn = document.getElementById('stepperRight');
            
            leftBtn.classList.remove('active');
            stopBtn.classList.remove('active');
            rightBtn.classList.remove('active');
            
            if (stepperCommand === 1) {
                leftBtn.classList.add('active');
            } else if (stepperCommand === 2) {
                rightBtn.classList.add('active');
            } else {
                stopBtn.classList.add('active');
            }
        }
        
        function updateStatus(message, className) {
            const status = document.getElementById('status');
            status.textContent = message;
            status.className = 'status-bar ' + className;
        }
        
        // Ползунки
        function setupSlider(sliderId, valueId, configKey, onChange) {
            const slider = document.getElementById(sliderId);
            const valueSpan = document.getElementById(valueId);
            
            slider.addEventListener('input', function() {
                const val = parseFloat(this.value);
                valueSpan.textContent = val.toFixed(2);
                if (onChange) onChange(val);
                
                const data = {};
                data[configKey] = val;
                fetch('/config', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                }).catch(err => console.error('Config error:', err));
            });
        }
        
        setupSlider('maxLinear', 'maxLinearVal', 'max_linear', (val) => { currentMaxLinear = val; });
        setupSlider('maxAngular', 'maxAngularVal', 'max_angular', (val) => { currentMaxAngular = val; });
        setupSlider('deadzone', 'deadzoneVal', 'angular_deadzone');
        
        // Клавиатура
        const keys = { ArrowUp: false, ArrowDown: false, ArrowLeft: false, ArrowRight: false };
        window.addEventListener('keydown', function(e) {
            if (e.key in keys) { e.preventDefault(); keys[e.key] = true; updateKeyboard(); }
        });
        window.addEventListener('keyup', function(e) {
            if (e.key in keys) { e.preventDefault(); keys[e.key] = false; updateKeyboard(); }
        });
        
        function updateKeyboard() {
            let linear = 0, angular = 0;
            if (keys.ArrowUp) linear += currentMaxLinear;
            if (keys.ArrowDown) linear -= currentMaxLinear;
            if (keys.ArrowLeft) angular += currentMaxAngular;
            if (keys.ArrowRight) angular -= currentMaxAngular;
            
            linear = parseFloat(linear.toFixed(2));
            angular = parseFloat(angular.toFixed(2));
            
            updateSpeeds(linear, angular);
            sendCmdVel(linear, angular);
        }
        
        // Лидар
        function drawLidar(scanData) {
            const container = canvas.parentElement;
            const size = Math.min(container.clientWidth, container.clientHeight - 60);
            canvas.width = size;
            canvas.height = size;
            
            const centerX = canvas.width / 2;
            const centerY = canvas.height / 2;
            const maxRadius = Math.min(centerX, centerY) - 15;
            
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            let autoScale = scanData.range_max || 10.0;
            if (scanData.ranges && scanData.ranges.length > 0) {
                const validRanges = scanData.ranges.filter(r => r !== null && isFinite(r));
                if (validRanges.length > 0) {
                    const maxRange = Math.max(...validRanges);
                    autoScale = Math.max(maxRange * 1.2, 2.0);
                }
            }
            
            ctx.strokeStyle = '#1a1a1a';
            ctx.lineWidth = 1;
            for (let i = 1; i <= 4; i++) {
                const radius = (maxRadius / 4) * i;
                ctx.beginPath();
                ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
                ctx.stroke();
            }
            
            ctx.beginPath();
            ctx.moveTo(centerX - maxRadius, centerY);
            ctx.lineTo(centerX + maxRadius, centerY);
            ctx.moveTo(centerX, centerY - maxRadius);
            ctx.lineTo(centerX, centerY + maxRadius);
            ctx.stroke();
            
            if (!scanData.ranges || scanData.ranges.length === 0) return;
            
            const angleMin = scanData.angle_min;
            const angleIncrement = scanData.angle_increment;
            
            let minDist = Infinity, maxDist = 0, validPoints = 0;
            
            ctx.fillStyle = '#ff6600';
            for (let i = 0; i < scanData.ranges.length; i++) {
                const range = scanData.ranges[i];
                if (range === null || !isFinite(range)) continue;
                
                const angle = angleMin + i * angleIncrement;
                const x = centerX + (range / autoScale) * maxRadius * Math.sin(angle);
                const y = centerY - (range / autoScale) * maxRadius * Math.cos(angle);
                
                ctx.beginPath();
                ctx.arc(x, y, 2, 0, 2 * Math.PI);
                ctx.fill();
                
                if (range < minDist) minDist = range;
                if (range > maxDist) maxDist = range;
                validPoints++;
            }
            
            ctx.fillStyle = '#ffffff';
            ctx.strokeStyle = '#ff6600';
            ctx.lineWidth = 2;
            const robotSize = 12;
            ctx.beginPath();
            ctx.moveTo(centerX, centerY - robotSize);
            ctx.lineTo(centerX - robotSize * 0.7, centerY + robotSize * 0.5);
            ctx.lineTo(centerX + robotSize * 0.7, centerY + robotSize * 0.5);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            
            document.getElementById('minDist').textContent = 
                minDist === Infinity ? '--' : minDist.toFixed(2);
            document.getElementById('maxDist').textContent = 
                maxDist === 0 ? '--' : maxDist.toFixed(2);
            document.getElementById('pointsCount').textContent = validPoints;
        }
        
        function fetchLidarData() {
            fetch('/scan')
                .then(response => response.json())
                .then(data => drawLidar(data))
                .catch(error => console.error('Error fetching lidar data:', error));
        }
        
        // Загрузка конфигурации
        fetch('/config')
            .then(response => response.json())
            .then(cfg => {
                currentMaxLinear = cfg.max_linear;
                currentMaxAngular = cfg.max_angular;
                document.getElementById('maxLinear').value = cfg.max_linear;
                document.getElementById('maxLinearVal').textContent = cfg.max_linear.toFixed(2);
                document.getElementById('maxAngular').value = cfg.max_angular;
                document.getElementById('maxAngularVal').textContent = cfg.max_angular.toFixed(2);
                document.getElementById('deadzone').value = cfg.angular_deadzone;
                document.getElementById('deadzoneVal').textContent = cfg.angular_deadzone.toFixed(2);
            })
            .catch(() => {});
        
        fetch('/motor/status')
            .then(response => response.json())
            .then(result => { motorEnabled = result.enabled; updateMotorUI(); })
            .catch(() => {});
        
        fetch('/stepper/status')
            .then(response => response.json())
            .then(result => { stepperCommand = result.command; updateStepperUI(); })
            .catch(() => {});
        
        updateStatus('Ready', 'connected');
        updateStepperUI();
        setInterval(fetchLidarData, 100);
        fetchLidarData();
    </script>
</body>
</html>
"""

def main(args=None):
    rclpy.init(args=args)
    node = WebTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()