#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
import heapq
from math import atan2
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
    q.z = np.sin(yaw / 2.0)
    q.w = np.cos(yaw / 2.0)
    return q

class AStarPlanner(Node):
    def __init__(self):
        super().__init__("a_star_planner")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter("map_topic", "/map_inflated")
        map_topic = self.get_parameter("map_topic").value
        
        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, map_qos)
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        
        # Используем кастомный сервис
        self.get_path_srv = self.create_service(
            GetPathToPoint, 
            "get_path_to_point", 
            self.get_path_callback
        )
        
        self.map_ = None
        self.map_lock = threading.Lock()
        
        self.get_logger().info("A* Planner (Differential Drive) initialized.")
        self.get_logger().info("Сервис доступен: /get_path_to_point")

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
            # Трансформируем цель в систему координат карты
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

        # Получаем текущую позицию робота через TF
        try:
            robot_in_map = self.tf_buffer.lookup_transform(
                map_frame,
                "base_footprint",  # или "base_link"
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"Не удалось получить позицию робота: {e}")
            response.success = False
            response.path = Path()
            return response

        # Конвертируем в Pose
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

        # 1. Поиск пути A*
        raw_path_grid = self.astar(start_grid, goal_grid, current_map)
        if not raw_path_grid:
            self.get_logger().warn("Путь не найден.")
            response.success = False
            response.path = Path()
            return response

        # 2. Упрощение пути (Line-of-Sight)
        smoothed_grid = self.smooth_path_los(raw_path_grid, current_map)
        
        # 3. Формирование Path с касательными ориентациями
        path_msg = Path()
        path_msg.header.frame_id = map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(len(smoothed_grid)):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose = self.grid_to_world(smoothed_grid[i], current_map.info)
            
            if i < len(smoothed_grid) - 1:
                # Ориентация по касательной к следующему сегменту
                next_node = smoothed_grid[i+1]
                dx = next_node[0] - smoothed_grid[i][0]
                dy = next_node[1] - smoothed_grid[i][1]
                yaw = atan2(dy, dx)
                ps.pose.orientation = yaw_to_quat(yaw)
            else:
                # Для последней точки берем ориентацию из цели
                goal_yaw = quat_to_yaw(goal_in_map.pose.orientation)
                if abs(goal_yaw) < 1e-3 and goal_in_map.pose.orientation.w > 0.99:
                    prev_node = smoothed_grid[i-1]
                    dx = smoothed_grid[i][0] - prev_node[0]
                    dy = smoothed_grid[i][1] - prev_node[1]
                    yaw = atan2(dy, dx)
                    ps.pose.orientation = yaw_to_quat(yaw)
                else:
                    ps.pose.orientation = goal_in_map.pose.orientation
            
            path_msg.poses.append(ps)
            
        self.path_pub.publish(path_msg)
        response.path = path_msg
        response.success = True
        self.get_logger().info(f"Путь построен: {len(path_msg.poses)} точек.")
        return response

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