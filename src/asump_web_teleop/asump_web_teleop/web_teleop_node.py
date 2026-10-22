import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from flask import Flask, render_template_string, jsonify, request
from flask_cors import CORS
import threading
import socket
import math

class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop_node')
        
        # Публикуем в /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Подписываемся на /scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Клиент для сервиса мотора
        self.motor_client = self.create_client(SetBool, '/set_aux_motor')
        self.motor_state = False
        
        # Текущие значения скорости
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Данные лидара
        self.scan_data = {
            'ranges': [],
            'angle_min': 0.0,
            'angle_max': 0.0,
            'angle_increment': 0.0,
            'range_min': 0.0,
            'range_max': 0.0
        }
        
        # Создаем Flask приложение
        self.app = Flask(__name__)
        CORS(self.app)
        
        # Маршруты
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        @self.app.route('/cmd_vel', methods=['POST'])
        def cmd_vel():
            try:
                data = request.json
                if data is None:
                    return jsonify({'status': 'error', 'message': 'No JSON data'}), 400
                
                linear = float(data.get('linear', 0.0) or 0.0)
                angular = float(data.get('angular', 0.0) or 0.0)
                
                self.current_linear = max(-2.0, min(2.0, linear))
                self.current_angular = max(-2.0, min(2.0, angular))
                
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
                
                if not self.motor_client.wait_for_service(timeout_sec=1.0):
                    return jsonify({'status': 'error', 'message': 'Service not available'}), 503
                
                req = SetBool.Request()
                req.data = enable
                future = self.motor_client.call_async(req)
                
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result() is not None:
                    self.motor_state = enable
                    self.get_logger().info(f'Motor {"enabled" if enable else "disabled"}')
                    return jsonify({
                        'status': 'ok', 
                        'enabled': enable,
                        'message': future.result().message
                    })
                else:
                    return jsonify({'status': 'error', 'message': 'Service call failed'}), 500
                    
            except Exception as e:
                self.get_logger().error(f'Error in /motor: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @self.app.route('/motor/status')
        def motor_status():
            return jsonify({'enabled': self.motor_state})
        
        @self.app.route('/scan')
        def scan():
            return jsonify(self.scan_data)
        
        # Получаем IP-адрес
        self.ip_address = self.get_ip_address()
        self.port = 5000
        
        self.get_logger().info(f'Web Teleop Node запущена!')
        self.get_logger().info(f'Откройте в браузере: http://{self.ip_address}:{self.port}')
        
        # Запускаем Flask в отдельном потоке
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        
        # Таймер для публикации cmd_vel
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
    
    def scan_callback(self, msg):
        """Обработчик данных лидара"""
        try:
            # Фильтруем бесконечные и NaN значения
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
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Courier New', monospace;
            background: #0a0a0a;
            color: #ffffff;
            padding: 20px;
            min-height: 100vh;
        }
        
        .container {
            max-width: 800px;
            margin: 0 auto;
        }
        
        .header {
            border-bottom: 2px solid #ff6600;
            padding-bottom: 15px;
            margin-bottom: 30px;
        }
        
        h1 {
            font-size: 24px;
            font-weight: bold;
            color: #ff6600;
            text-transform: uppercase;
            letter-spacing: 2px;
        }
        
        .subtitle {
            font-size: 12px;
            color: #888;
            margin-top: 5px;
        }
        
        .section {
            background: #1a1a1a;
            border: 1px solid #333;
            padding: 20px;
            margin-bottom: 20px;
        }
        
        .section-title {
            font-size: 14px;
            color: #ff6600;
            text-transform: uppercase;
            margin-bottom: 15px;
            letter-spacing: 1px;
        }
        
        #joystick {
            width: 220px;
            height: 220px;
            margin: 20px auto;
            background: #0a0a0a;
            border: 2px solid #333;
            position: relative;
        }
        
        .values {
            background: #0a0a0a;
            border: 1px solid #333;
            padding: 15px;
            margin: 15px 0;
            font-family: 'Courier New', monospace;
        }
        
        .value-row {
            display: flex;
            justify-content: space-between;
            padding: 5px 0;
            font-size: 14px;
        }
        
        .value-label {
            color: #888;
        }
        
        .value-number {
            color: #ff6600;
            font-weight: bold;
        }
        
        .motor-control {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 15px;
            background: #0a0a0a;
            border: 1px solid #333;
        }
        
        .motor-status {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .status-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #333;
            border: 2px solid #555;
        }
        
        .status-indicator.active {
            background: #ff6600;
            border-color: #ff8533;
        }
        
        .status-text {
            font-size: 14px;
            color: #888;
        }
        
        .motor-btn {
            background: #ff6600;
            color: #000;
            border: none;
            padding: 10px 25px;
            font-size: 14px;
            font-weight: bold;
            font-family: 'Courier New', monospace;
            text-transform: uppercase;
            cursor: pointer;
            letter-spacing: 1px;
            transition: all 0.2s;
        }
        
        .motor-btn:hover {
            background: #ff8533;
        }
        
        .motor-btn:active {
            background: #cc5200;
        }
        
        .motor-btn.active {
            background: #333;
            color: #ff6600;
            border: 1px solid #ff6600;
        }
        
        .stop-btn {
            width: 100%;
            background: #ff6600;
            color: #000;
            border: none;
            padding: 15px;
            font-size: 16px;
            font-weight: bold;
            font-family: 'Courier New', monospace;
            text-transform: uppercase;
            cursor: pointer;
            letter-spacing: 2px;
            margin-top: 15px;
        }
        
        .stop-btn:hover {
            background: #ff8533;
        }
        
        .stop-btn:active {
            background: #cc5200;
        }
        
        .status-bar {
            margin-top: 20px;
            padding: 10px;
            background: #0a0a0a;
            border: 1px solid #333;
            font-size: 12px;
            color: #888;
            text-align: center;
        }
        
        .status-bar.connected {
            color: #ff6600;
            border-color: #ff6600;
        }
        
        .status-bar.error {
            color: #ff3333;
            border-color: #ff3333;
        }
        
        #lidarCanvas {
            display: block;
            margin: 20px auto;
            background: #0a0a0a;
            border: 2px solid #333;
        }
        
        .lidar-info {
            display: flex;
            justify-content: space-around;
            margin-top: 15px;
            font-size: 12px;
        }
        
        .lidar-stat {
            text-align: center;
        }
        
        .lidar-stat-value {
            color: #ff6600;
            font-weight: bold;
            font-size: 16px;
        }
        
        .lidar-stat-label {
            color: #888;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Robot Control</h1>
            <div class="subtitle">Teleoperation Interface v1.1</div>
        </div>
        
        <div class="section">
            <div class="section-title">Lidar Scan</div>
            <canvas id="lidarCanvas" width="400" height="400"></canvas>
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
        
        <div class="section">
            <div class="section-title">Movement Control</div>
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
        
        <div class="section">
            <div class="section-title">Auxiliary Motor</div>
            <div class="motor-control">
                <div class="motor-status">
                    <div class="status-indicator" id="motorIndicator"></div>
                    <span class="status-text" id="motorText">OFF</span>
                </div>
                <button class="motor-btn" id="motorBtn" onclick="toggleMotor()">Enable</button>
            </div>
        </div>
        
        <div class="status-bar" id="status">Initializing...</div>
    </div>
    
    <script>
        const MAX_LINEAR = 1.0;
        const MAX_ANGULAR = 1.0;
        let motorEnabled = false;
        
        // Canvas для лидара
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = Math.min(centerX, centerY) - 20;
        
        // Джойстик
        const joystick = nipplejs.create({
            zone: document.getElementById('joystick'),
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: '#ff6600',
            size: 140
        });
        
        joystick.on('move', function(evt, data) {
            const linear = parseFloat((data.vector.y * MAX_LINEAR).toFixed(2));
            const angular = parseFloat((-data.vector.x * MAX_ANGULAR).toFixed(2));
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
            const data = {
                linear: Number(linear) || 0.0,
                angular: Number(angular) || 0.0
            };
            
            fetch('/cmd_vel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(data)
            })
            .then(response => response.json())
            .then(() => updateStatus('Connected', 'connected'))
            .catch(() => updateStatus('Connection Error', 'error'));
        }
        
        function stopRobot() {
            updateSpeeds(0, 0);
            sendCmdVel(0.0, 0.0);
        }
        
        // Управление мотором
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
                } else {
                    alert('Motor control failed: ' + result.message);
                }
            })
            .catch(error => {
                alert('Error: ' + error.message);
            });
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
        
        function updateStatus(message, className) {
            const status = document.getElementById('status');
            status.textContent = message;
            status.className = 'status-bar ' + className;
        }
        
        // Клавиатура
        const keys = {
            ArrowUp: false,
            ArrowDown: false,
            ArrowLeft: false,
            ArrowRight: false
        };
        
        window.addEventListener('keydown', function(e) {
            if (e.key in keys) {
                e.preventDefault();
                keys[e.key] = true;
                updateKeyboard();
            }
        });
        
        window.addEventListener('keyup', function(e) {
            if (e.key in keys) {
                e.preventDefault();
                keys[e.key] = false;
                updateKeyboard();
            }
        });
        
        function updateKeyboard() {
            let linear = 0;
            let angular = 0;
            
            if (keys.ArrowUp) linear += MAX_LINEAR;
            if (keys.ArrowDown) linear -= MAX_LINEAR;
            if (keys.ArrowLeft) angular += MAX_ANGULAR;
            if (keys.ArrowRight) angular -= MAX_ANGULAR;
            
            linear = parseFloat(linear.toFixed(2));
            angular = parseFloat(angular.toFixed(2));
            
            updateSpeeds(linear, angular);
            sendCmdVel(linear, angular);
        }
        
        // Визуализация лидара
        function drawLidar(scanData) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Рисуем сетку
            ctx.strokeStyle = '#1a1a1a';
            ctx.lineWidth = 1;
            
            // Концентрические круги
            for (let i = 1; i <= 4; i++) {
                const radius = (maxRadius / 4) * i;
                ctx.beginPath();
                ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
                ctx.stroke();
            }
            
            // Крест
            ctx.beginPath();
            ctx.moveTo(centerX - maxRadius, centerY);
            ctx.lineTo(centerX + maxRadius, centerY);
            ctx.moveTo(centerX, centerY - maxRadius);
            ctx.lineTo(centerX, centerY + maxRadius);
            ctx.stroke();
            
            if (!scanData.ranges || scanData.ranges.length === 0) {
                return;
            }
            
            const angleMin = scanData.angle_min;
            const angleIncrement = scanData.angle_increment;
            const rangeMax = scanData.range_max;
            
            let minDist = Infinity;
            let maxDist = 0;
            let validPoints = 0;
            
            // Рисуем точки лидара
            ctx.fillStyle = '#ff6600';
            
            for (let i = 0; i < scanData.ranges.length; i++) {
                const range = scanData.ranges[i];
                
                if (range === null || !isFinite(range)) {
                    continue;
                }
                
                const angle = angleMin + i * angleIncrement;
                
                // Преобразуем полярные координаты в декартовы
                // Лидар: 0 - вперед, по часовой стрелке
                // Canvas: 0 - вправо, против часовой стрелки
                const x = centerX + (range / rangeMax) * maxRadius * Math.sin(angle);
                const y = centerY - (range / rangeMax) * maxRadius * Math.cos(angle);
                
                ctx.beginPath();
                ctx.arc(x, y, 2, 0, 2 * Math.PI);
                ctx.fill();
                
                // Статистика
                if (range < minDist) minDist = range;
                if (range > maxDist) maxDist = range;
                validPoints++;
            }
            
            // Рисуем робота (треугольник)
            ctx.fillStyle = '#ffffff';
            ctx.strokeStyle = '#ff6600';
            ctx.lineWidth = 2;
            
            const robotSize = 15;
            ctx.beginPath();
            ctx.moveTo(centerX, centerY - robotSize); // Вершина (вперед)
            ctx.lineTo(centerX - robotSize * 0.7, centerY + robotSize * 0.5);
            ctx.lineTo(centerX + robotSize * 0.7, centerY + robotSize * 0.5);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            
            // Обновляем статистику
            document.getElementById('minDist').textContent = 
                minDist === Infinity ? '--' : minDist.toFixed(2);
            document.getElementById('maxDist').textContent = 
                maxDist === 0 ? '--' : maxDist.toFixed(2);
            document.getElementById('pointsCount').textContent = validPoints;
        }
        
        // Периодический опрос данных лидара
        function fetchLidarData() {
            fetch('/scan')
                .then(response => response.json())
                .then(data => {
                    drawLidar(data);
                })
                .catch(error => {
                    console.error('Error fetching lidar data:', error);
                });
        }
        
        // Проверка статуса мотора при загрузке
        fetch('/motor/status')
            .then(response => response.json())
            .then(result => {
                motorEnabled = result.enabled;
                updateMotorUI();
            })
            .catch(() => {});
        
        updateStatus('Ready', 'connected');
        
        // Запускаем опрос лидара каждые 100мс
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