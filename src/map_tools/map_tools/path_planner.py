#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
import heapq
from math import atan2, sqrt
import numpy as np
import threading
from collections import deque

# Импортируем кастомный сервис
from asump_localization.srv import GetPathToPoint

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = float(np.sin(yaw / 2.0))
    q.w = float(np.cos(yaw / 2.0))
    return q


class AStarPlanner(Node):
    def __init__(self):
        super().__init__("a_star_planner")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter("map_topic", "/map_inflated")
        self.declare_parameter("interpolation_distance", 0.15)  # метров между точками
        self.declare_parameter("smoothing_alpha", 0.5)          # сила Catmull-Rom (0..1)
        self.declare_parameter("arrow_scale", 0.3)              # длина стрелки визуализации
        
        map_topic = self.get_parameter("map_topic").value
        self.interpolation_distance = self.get_parameter("interpolation_distance").value
        self.smoothing_alpha = self.get_parameter("smoothing_alpha").value
        self.arrow_scale = self.get_parameter("arrow_scale").value
        
        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, map_qos)
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        
        # НОВЫЙ издатель для визуализации ориентаций (стрелки в RViz)
        self.orientations_pub = self.create_publisher(MarkerArray, "/plan/orientations", 10)
        
        self.get_path_srv = self.create_service(
            GetPathToPoint, 
            "get_path_to_point", 
            self.get_path_callback
        )
        
        self.map_ = None
        self.map_lock = threading.Lock()
        
        self.get_logger().info("A* Planner (Differential Drive) initialized.")
        self.get_logger().info("Сервис: /get_path_to_point")
        self.get_logger().info("Визуализация ориентаций: /plan/orientations")

    def map_callback(self, map_msg: OccupancyGrid):
        with self.map_lock:
            self.map_ = map_msg

    def get_path_callback(self, request, response):
        with self.map_lock:
            current_map = self.map_
            
        if current_map is None:
            self.get_logger().warn("Карта еще не получена!")
            response.success = False
            response.path = Path()
            return response

        map_frame = current_map.header.frame_id
        
        try:
            goal_in_map = self.tf_buffer.transform(
                request.goal_pose, 
                map_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            ) if request.goal_pose.header.frame_id != map_frame else request.goal_pose
        except Exception as e:
            self.get_logger().error(f"Ошибка TF преобразования: {e}")
            response.success = False
            response.path = Path()
            return response

        try:
            robot_in_map = self.tf_buffer.lookup_transform(
                map_frame,
                "base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"Не удалось получить позицию робота: {e}")
            response.success = False
            response.path = Path()
            return response

        start_pose = Pose()
        start_pose.position.x = robot_in_map.transform.translation.x
        start_pose.position.y = robot_in_map.transform.translation.y
        start_pose.orientation = robot_in_map.transform.rotation

        start_grid = self.world_to_grid(start_pose, current_map.info)
        goal_grid = self.world_to_grid(goal_in_map.pose, current_map.info)
        
        if not self.is_valid(start_grid, current_map):
            start_grid = self.find_nearest_free(start_grid, current_map)
            if start_grid is None:
                self.get_logger().error("Старт в препятствии, свободных клеток рядом нет.")
                response.success = False
                response.path = Path()
                return response
                
        if not self.is_valid(goal_grid, current_map):
            self.get_logger().error("Цель находится внутри препятствия.")
            response.success = False
            response.path = Path()
            return response

        # 1. A* поиск
        raw_path_grid = self.astar(start_grid, goal_grid, current_map)
        if not raw_path_grid:
            self.get_logger().warn("Путь не найден.")
            response.success = False
            response.path = Path()
            return response

        # 2. Line-of-Sight упрощение (убираем зигзаги сетки)
        smoothed_grid = self.smooth_path_los(raw_path_grid, current_map)
        
        # 3. Конвертируем в мировые координаты
        world_points = [self.grid_to_world(n, current_map.info) for n in smoothed_grid]
        
        # 4. === СГЛАЖИВАНИЕ ЧЕРЕЗ CATMULL-ROM СПЛАЙН ===
        smoothed_points = self.catmull_rom_smooth(world_points)
        
        # 5. === ИНТЕРПОЛЯЦИЯ ТОЧЕК (плотная последовательность) ===
        interpolated_points = self.interpolate_points(smoothed_points)
        
        # 6. Формируем Path с касательными ориентациями
        path_msg = Path()
        path_msg.header.frame_id = map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Сохраняем ориентации для визуализации (до прореживания)
        all_orientations = []
        
        for i in range(len(interpolated_points)):
            pose = Pose()
            pose.position.x = interpolated_points[i][0]
            pose.position.y = interpolated_points[i][1]
            pose.position.z = 0.0
            
            if i < len(interpolated_points) - 1:
                dx = interpolated_points[i+1][0] - interpolated_points[i][0]
                dy = interpolated_points[i+1][1] - interpolated_points[i][1]
                yaw = atan2(dy, dx)
            else:
                # Последняя точка: смотрим назад (касательная последнего сегмента)
                dx = interpolated_points[i][0] - interpolated_points[i-1][0]
                dy = interpolated_points[i][1] - interpolated_points[i-1][1]
                yaw = atan2(dy, dx)
            
            pose.orientation = yaw_to_quat(yaw)
            all_orientations.append((pose, yaw))
            
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose = pose
            path_msg.poses.append(ps)
        
        # Переопределяем ориентацию последней точки из цели (если задана)
        goal_yaw = quat_to_yaw(goal_in_map.pose.orientation)
        if not (abs(goal_yaw) < 1e-3 and goal_in_map.pose.orientation.w > 0.99):
            path_msg.poses[-1].pose.orientation = goal_in_map.pose.orientation
        
        # 7. Публикуем путь
        self.path_pub.publish(path_msg)
        
        # 8. === ПУБЛИКАЦИЯ ВИЗУАЛИЗАЦИИ ОРИЕНТАЦИЙ ===
        self.publish_orientations_markers(map_frame, all_orientations)
        
        response.path = path_msg
        response.success = True
        self.get_logger().info(
            f"Путь построен: {len(path_msg.poses)} точек "
            f"(после сглаживания из {len(raw_path_grid)} исходных)."
        )
        return response

    # ==================== СГЛАЖИВАНИЕ: Catmull-Rom ====================
    def catmull_rom_smooth(self, points, num_subdivisions=4):
        """
        Catmull-Rom spline сглаживание.
        num_subdivisions — сколько промежуточных точек генерируем между каждой парой.
        """
        if len(points) < 3:
            # Слишком мало точек для сплайна — возвращаем как есть
            return [(p.position.x, p.position.y) for p in points]
        
        # Дополняем крайние точки для корректной работы алгоритма
        # (дублируем первую и последнюю)
        extended = [points[0]] + list(points) + [points[-1]]
        
        smoothed = []
        
        for i in range(1, len(extended) - 2):
            p0 = extended[i - 1]
            p1 = extended[i]
            p2 = extended[i + 1]
            p3 = extended[i + 2]
            
            for j in range(num_subdivisions):
                t = j / num_subdivisions
                t2 = t * t
                t3 = t2 * t
                
                # Catmull-Rom формула
                x = 0.5 * (
                    (2.0 * p1.position.x) +
                    (-p0.position.x + p2.position.x) * t +
                    (2.0 * p0.position.x - 5.0 * p1.position.x + 4.0 * p2.position.x - p3.position.x) * t2 +
                    (-p0.position.x + 3.0 * p1.position.x - 3.0 * p2.position.x + p3.position.x) * t3
                )
                y = 0.5 * (
                    (2.0 * p1.position.y) +
                    (-p0.position.y + p2.position.y) * t +
                    (2.0 * p0.position.y - 5.0 * p1.position.y + 4.0 * p2.position.y - p3.position.y) * t2 +
                    (-p0.position.y + 3.0 * p1.position.y - 3.0 * p2.position.y + p3.position.y) * t3
                )
                
                smoothed.append((x, y))
        
        # Добавляем последнюю точку
        smoothed.append((points[-1].position.x, points[-1].position.y))
        
        return smoothed

    # ==================== ИНТЕРПОЛЯЦИЯ ТОЧЕК ====================
    def interpolate_points(self, points):
        """
        Создаёт плотную последовательность точек с заданным шагом.
        Это важно для Pure Pursuit контроллера.
        """
        if len(points) < 2:
            return points
        
        interpolated = [points[0]]
        
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            
            dx = x1 - x0
            dy = y1 - y0
            seg_length = sqrt(dx * dx + dy * dy)
            
            if seg_length < 1e-6:
                continue
            
            num_steps = max(1, int(seg_length / self.interpolation_distance))
            
            for step in range(1, num_steps + 1):
                t = step / num_steps
                ix = x0 + t * dx
                iy = y0 + t * dy
                interpolated.append((ix, iy))
        
        return interpolated

    # ==================== ВИЗУАЛИЗАЦИЯ ОРИЕНТАЦИЙ ====================
    def publish_orientations_markers(self, frame_id, orientations):
        """
        Публикует MarkerArray со стрелками, показывающими ориентацию робота
        в каждой точке пути. Прореживаем для читаемости.
        """
        marker_array = MarkerArray()
        
        # Сначала очищаем предыдущие маркеры (DELETE all)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = "path_orientations"
        marker_array.markers.append(delete_marker)
        
        # Прореживание: показываем каждую N-ную стрелку
        step = max(1, len(orientations) // 30)  # ~30 стрелок на весь путь
        
        for idx in range(0, len(orientations), step):
            pose, yaw = orientations[idx]
            
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path_orientations"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Позиция стрелки
            marker.pose.position.x = pose.position.x
            marker.pose.position.y = pose.position.y
            marker.pose.position.z = 0.05  # чуть выше пола для видимости
            marker.pose.orientation = pose.orientation
            
            # Размер стрелки: длина, ширина стержня, ширина наконечника
            marker.scale.x = self.arrow_scale       # длина стрелки
            marker.scale.y = 0.05                   # ширина стержня
            marker.scale.z = 0.05                   # высота
            
            # Градиент цвета: от зелёного (начало) к синему (конец)
            t = idx / max(1, len(orientations) - 1)
            marker.color.r = 0.0
            marker.color.g = float(1.0 - t)
            marker.color.b = float(t)
            marker.color.a = 0.9
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            
            marker_array.markers.append(marker)
        
        # Добавляем отдельный маркер для цели (красная стрелка)
        if orientations:
            goal_marker = Marker()
            goal_marker.header.frame_id = frame_id
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "path_orientations"
            goal_marker.id = 999999
            goal_marker.type = Marker.ARROW
            goal_marker.action = Marker.ADD
            
            last_pose, _ = orientations[-1]
            goal_marker.pose.position.x = last_pose.position.x
            goal_marker.pose.position.y = last_pose.position.y
            goal_marker.pose.position.z = 0.1
            goal_marker.pose.orientation = last_pose.orientation
            
            goal_marker.scale.x = self.arrow_scale * 1.5
            goal_marker.scale.y = 0.08
            goal_marker.scale.z = 0.08
            
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.2
            goal_marker.color.b = 0.2
            goal_marker.color.a = 1.0
            
            marker_array.markers.append(goal_marker)
        
        self.orientations_pub.publish(marker_array)
        self.get_logger().debug(f"Опубликовано {len(marker_array.markers) - 1} маркеров ориентации")

    # ==================== A* (без изменений) ====================
    def astar(self, start, goal, map_msg):
        movements = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
                
            for dx, dy, cost in movements:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid(neighbor, map_msg):
                    continue
                    
                if dx != 0 and dy != 0:
                    if not self.is_valid((current[0] + dx, current[1]), map_msg) or \
                       not self.is_valid((current[0], current[1] + dy), map_msg):
                        continue
                
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return []

    def heuristic(self, node, goal):
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        return (dx + dy) + (1.414 - 2) * min(dx, dy)

    def smooth_path_los(self, path, map_msg):
        if len(path) <= 2:
            return path
            
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            farthest_valid_idx = current_idx + 1
            
            for i in range(len(path) - 1, current_idx, -1):
                if self.has_line_of_sight(path[current_idx], path[i], map_msg):
                    farthest_valid_idx = i
                    break
                    
            smoothed.append(path[farthest_valid_idx])
            current_idx = farthest_valid_idx
            
        return smoothed

    def has_line_of_sight(self, n1, n2, map_msg):
        x0, y0 = n1
        x1, y1 = n2
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            if not self.is_valid((x0, y0), map_msg):
                return False
                
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return True

    def is_valid(self, node, map_msg):
        x, y = node
        if x < 0 or x >= map_msg.info.width or y < 0 or y >= map_msg.info.height:
            return False
        idx = y * map_msg.info.width + x
        return map_msg.data[idx] == 0

    def find_nearest_free(self, start, map_msg):
        queue = deque([start])
        visited = {start}
        
        while queue:
            curr = queue.popleft()
            if self.is_valid(curr, map_msg):
                return curr
                
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0: continue
                    nxt = (curr[0] + dx, curr[1] + dy)
                    if nxt not in visited:
                        visited.add(nxt)
                        if 0 <= nxt[0] < map_msg.info.width and 0 <= nxt[1] < map_msg.info.height:
                            queue.append(nxt)
        return None

    def world_to_grid(self, pose, info):
        x = int((pose.position.x - info.origin.position.x) / info.resolution)
        y = int((pose.position.y - info.origin.position.y) / info.resolution)
        return (x, y)

    def grid_to_world(self, node, info):
        pose = Pose()
        pose.position.x = node[0] * info.resolution + info.origin.position.x
        pose.position.y = node[1] * info.resolution + info.origin.position.y
        pose.position.z = 0.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()