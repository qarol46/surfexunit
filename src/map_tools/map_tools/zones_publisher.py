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

        self.declare_parameter('charging_station.enabled', True)
        self.declare_parameter('charging_station.name', 'charging_station')
        self.declare_parameter('charging_station.x', 0.0)
        self.declare_parameter('charging_station.y', 0.0)
        self.declare_parameter('charging_station.yaw', 0.0)
        self.declare_parameter('charging_station.radius', 0.5)

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

        publish_rate = self.get_parameter('publish_rate').value
        if publish_rate <= 0.0:
            publish_rate = 1.0

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_zones)

        # Сразу публикуем, чтобы RViz мог подхватить latched-сообщения.
        self.publish_zones()

        self.get_logger().info('Zones publisher started')

    def publish_zones(self):
        frame_id = self.get_parameter('frame_id').value
        now = self.get_clock().now().to_msg()

        marker_array = MarkerArray()

        enabled = self.get_parameter('charging_station.enabled').value

        if enabled:
            name = self.get_parameter('charging_station.name').value
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

        self.marker_pub.publish(marker_array)


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