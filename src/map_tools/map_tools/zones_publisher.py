import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import (
    Point,
    Point32,
    PolygonStamped,
    PoseStamped,
    Quaternion,
)
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class ZonesPublisher(Node):
    def __init__(self):
        super().__init__('zones_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)

        # Номер этажа: от него зависят координаты лестницы
        self.declare_parameter('floor', 1)

        self.declare_parameter('charging_station.enabled', True)
        self.declare_parameter('charging_station.name', 'charging_station')
        self.declare_parameter('charging_station.x', 0.0)
        self.declare_parameter('charging_station.y', 0.0)
        self.declare_parameter('charging_station.yaw', 0.0)
        self.declare_parameter('charging_station.radius', 0.5)

        self.declare_parameter('stairs.enabled', True)
        self.declare_parameter('stairs.name', 'stairs')
        self.declare_parameter('stairs.floor1.x', -4.5)
        self.declare_parameter('stairs.floor1.y', 6.0)
        self.declare_parameter('stairs.floor1.yaw', 0.0)
        self.declare_parameter('stairs.floor2.x', 0.0)
        self.declare_parameter('stairs.floor2.y', 0.0)
        self.declare_parameter('stairs.floor2.yaw', 0.0)

        # Запретные зоны: список имён + полигон каждой зоны
        # (плоский список вершин x1,y1,x2,y2,...)
        self.declare_parameter('no_go_zones.names', [''])

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/zones/markers',
            qos_latched,
        )

        self.polygon_pub = self.create_publisher(
            PolygonStamped,
            '/zones/charger_polygon',
            qos_latched,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/zones/charger_pose',
            qos_latched,
        )

        self.stairs_pose_pub = self.create_publisher(
            PoseStamped,
            '/zones/stairs_pose',
            qos_latched,
        )

        self.no_go_names = [
            n for n in self.get_parameter('no_go_zones.names').value if n
        ]
        self.no_go_pubs = {}
        for name in self.no_go_names:
            self.declare_parameter(f'no_go_zones.{name}.enabled', True)
            self.declare_parameter(f'no_go_zones.{name}.points', [0.0])
            self.no_go_pubs[name] = self.create_publisher(
                PolygonStamped,
                f'/zones/no_go/{name}',
                qos_latched,
            )

        publish_rate = self.get_parameter('publish_rate').value
        if publish_rate <= 0.0:
            publish_rate = 1.0

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_zones)

        # Сразу публикуем, чтобы RViz мог подхватить latched-сообщения.
        self.publish_zones()

        self.get_logger().info('Zones publisher started')

    def stairs_pose(self):
        floor = self.get_parameter('floor').value
        try:
            x = self.get_parameter(f'stairs.floor{floor}.x').value
            y = self.get_parameter(f'stairs.floor{floor}.y').value
            yaw = self.get_parameter(f'stairs.floor{floor}.yaw').value
        except rclpy.exceptions.ParameterNotDeclaredException:
            self.get_logger().warn(
                f'No stairs coordinates for floor {floor}, skipping')
            return None
        return x, y, yaw

    def publish_zones(self):
        frame_id = self.get_parameter('frame_id').value
        now = self.get_clock().now().to_msg()

        marker_array = MarkerArray()

        if self.get_parameter('charging_station.enabled').value:
            self.publish_charger(frame_id, now, marker_array)

        if self.get_parameter('stairs.enabled').value:
            self.publish_stairs(frame_id, now, marker_array)

        self.publish_no_go_zones(frame_id, now, marker_array)

        self.marker_pub.publish(marker_array)

    def publish_charger(self, frame_id, now, marker_array):
        x = self.get_parameter('charging_station.x').value
        y = self.get_parameter('charging_station.y').value
        yaw = self.get_parameter('charging_station.yaw').value
        radius = self.get_parameter('charging_station.radius').value

        if radius <= 0.0:
            radius = 0.01

        # 1. Marker for RViz
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = now

        marker.ns = 'zones'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation = yaw_to_quaternion(yaw)

        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = 0.1

        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.45

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        marker.frame_locked = True

        marker_array.markers.append(marker)

        # 2. PolygonStamped around charging station
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = frame_id
        polygon_msg.header.stamp = now

        n_points = 32

        for i in range(n_points):
            theta = 2.0 * math.pi * float(i) / float(n_points)
            p = Point32()
            p.x = x + radius * math.cos(theta)
            p.y = y + radius * math.sin(theta)
            p.z = 0.0
            polygon_msg.polygon.points.append(p)

        self.polygon_pub.publish(polygon_msg)

        # 3. Charger pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = now

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(yaw)

        self.pose_pub.publish(pose_msg)

    def publish_stairs(self, frame_id, now, marker_array):
        coords = self.stairs_pose()
        if coords is None:
            return
        x, y, yaw = coords

        # 1. Marker for RViz (оранжевый куб)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = now

        marker.ns = 'zones'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation = yaw_to_quaternion(yaw)

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.0
        marker.color.a = 0.6

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        marker.frame_locked = True

        marker_array.markers.append(marker)

        # 2. Stairs pose (prior для pose_initializer)
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = now

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(yaw)

        self.stairs_pose_pub.publish(pose_msg)

    def publish_no_go_zones(self, frame_id, now, marker_array):
        for idx, name in enumerate(self.no_go_names):
            if not self.get_parameter(f'no_go_zones.{name}.enabled').value:
                continue

            raw = self.get_parameter(f'no_go_zones.{name}.points').value
            if len(raw) < 6 or len(raw) % 2 != 0:
                self.get_logger().warn(
                    f'No-go zone "{name}": need >=3 vertices (flat '
                    f'x1,y1,... list), got {len(raw)} values, skipping')
                continue

            polygon_msg = PolygonStamped()
            polygon_msg.header.frame_id = frame_id
            polygon_msg.header.stamp = now

            for i in range(0, len(raw), 2):
                p = Point32()
                p.x = raw[i]
                p.y = raw[i + 1]
                p.z = 0.0
                polygon_msg.polygon.points.append(p)

            self.no_go_pubs[name].publish(polygon_msg)

            # Контур полигона для RViz (красный)
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now

            marker.ns = 'no_go_zones'
            marker.id = 100 + idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05

            marker.color.r = 0.95
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.9

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            marker.frame_locked = True

            for i in range(0, len(raw), 2):
                marker.points.append(Point(x=raw[i], y=raw[i + 1], z=0.05))
            # Замыкаем контур
            marker.points.append(
                Point(x=raw[0], y=raw[1], z=0.05))

            marker_array.markers.append(marker)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ZonesPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'zones_publisher error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
