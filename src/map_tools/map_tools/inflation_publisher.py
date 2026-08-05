import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from nav_msgs.msg import OccupancyGrid

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:
    distance_transform_edt = None


class InflationPublisher(Node):
    def __init__(self):
        super().__init__('inflation_publisher')

        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/map_inflated')
        self.declare_parameter('inflation_radius', 0.55)
        self.declare_parameter('inflate_unknown', False)

        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.inflate_unknown = self.get_parameter('inflate_unknown').value

        self.last_stamp = None

        out_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        in_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        output_topic = self.get_parameter('output_map_topic').value
        input_topic = self.get_parameter('input_map_topic').value

        self.pub = self.create_publisher(OccupancyGrid, output_topic, out_qos)

        # Основная подписка: хорошо работает с latched map_publisher/map_server.
        self.sub = self.create_subscription(
            OccupancyGrid,
            input_topic,
            self.map_callback,
            in_qos,
        )

        # Дополнительная подписка с sensor_data QoS на случай
        # publisher'ов вроде slam_toolbox.
        self.sub_sensor = self.create_subscription(
            OccupancyGrid,
            input_topic,
            self.map_callback,
            qos_profile_sensor_data,
        )

        if cv2 is None and distance_transform_edt is None:
            self.get_logger().warning(
                'OpenCV and SciPy were not found. '
                'Inflation will use approximate binary dilation.'
            )

        self.get_logger().info(
            f'Inflation publisher: {input_topic} -> {output_topic}, '
            f'radius={self.inflation_radius:.3f} m'
        )

    def map_callback(self, msg: OccupancyGrid):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        # Отсекаем дубли, если сообщение пришло через два разных QoS-подписчика.
        if stamp == self.last_stamp:
            return

        self.last_stamp = stamp

        try:
            inflated_msg = self.inflate(msg)
            self.pub.publish(inflated_msg)
        except Exception as e:
            self.get_logger().error(f'Inflation failed: {e}')

    def inflate(self, msg: OccupancyGrid) -> OccupancyGrid:
        if self.inflation_radius <= 0.0:
            return msg

        if msg.info.resolution <= 0.0:
            self.get_logger().warning('Map resolution is <= 0, returning original map.')
            return msg

        h = int(msg.info.height)
        w = int(msg.info.width)

        if h == 0 or w == 0:
            return msg

        expected = h * w
        if len(msg.data) != expected:
            self.get_logger().warning(
                'OccupancyGrid data size does not match width*height. '
                'Returning original map.'
            )
            return msg

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        occupied = grid >= 90
        if not np.any(occupied):
            return msg

        radius_px = int(math.ceil(self.inflation_radius / msg.info.resolution))
        if radius_px <= 0:
            return msg

        if cv2 is not None:
            out = self.inflate_with_cv2(grid, occupied, radius_px)
        elif distance_transform_edt is not None:
            out = self.inflate_with_scipy(grid, occupied, radius_px)
        else:
            out = self.inflate_binary_fallback(grid, occupied, radius_px)

        out_msg = OccupancyGrid()
        out_msg.header = msg.header
        out_msg.info = msg.info
        out_msg.data = out.ravel().astype(np.int8).tolist()

        return out_msg

    def inflate_with_cv2(
        self,
        grid: np.ndarray,
        occupied: np.ndarray,
        radius_px: int,
    ) -> np.ndarray:
        h, w = grid.shape

        # distanceTransform считает расстояние до ближайшего нуля.
        # Поэтому occupied делаем нулями, все остальное - единицами.
        src = np.ones((h, w), dtype=np.uint8)
        src[occupied] = 0

        dist = cv2.distanceTransform(src, cv2.DIST_L2, 5)

        out = grid.copy()

        mask = (dist > 0.0) & (dist <= float(radius_px))

        if not self.inflate_unknown:
            # Раздуваем только свободные клетки.
            mask &= (grid == 0)
        else:
            # Раздуваем свободные и unknown.
            mask &= (grid <= 0)

        cost = np.zeros_like(grid, dtype=np.float32)
        cost[mask] = 100.0 * (1.0 - dist[mask] / float(radius_px))

        out[mask] = np.clip(cost[mask], 0.0, 100.0).astype(np.int8)
        out[occupied] = 100

        return out

    def inflate_with_scipy(
        self,
        grid: np.ndarray,
        occupied: np.ndarray,
        radius_px: int,
    ) -> np.ndarray:
        # distance_transform_edt считает расстояние до ближайкого False.
        dist = distance_transform_edt(~occupied).astype(np.float32)

        out = grid.copy()

        mask = (dist > 0.0) & (dist <= float(radius_px))

        if not self.inflate_unknown:
            mask &= (grid == 0)
        else:
            mask &= (grid <= 0)

        cost = np.zeros_like(grid, dtype=np.float32)
        cost[mask] = 100.0 * (1.0 - dist[mask] / float(radius_px))

        out[mask] = np.clip(cost[mask], 0.0, 100.0).astype(np.int8)
        out[occupied] = 100

        return out

    def inflate_binary_fallback(
        self,
        grid: np.ndarray,
        occupied: np.ndarray,
        radius_px: int,
    ) -> np.ndarray:
        """
        Very simple approximate fallback.

        It grows occupied cells by 8-neighbor dilation.
        This is not a perfect circular inflation, but works without OpenCV/SciPy.
        """
        out = grid.copy()

        mask = occupied.copy()

        for _ in range(radius_px):
            new_mask = mask.copy()

            new_mask[1:, :] |= mask[:-1, :]
            new_mask[:-1, :] |= mask[1:, :]
            new_mask[:, 1:] |= mask[:, :-1]
            new_mask[:, :-1] |= mask[:, 1:]

            new_mask[1:, 1:] |= mask[:-1, :-1]
            new_mask[:-1, :-1] |= mask[1:, 1:]
            new_mask[1:, :-1] |= mask[:-1, 1:]
            new_mask[:-1, 1:] |= mask[1:, :-1]

            mask = new_mask

        inflate_mask = mask & ~occupied

        if not self.inflate_unknown:
            inflate_mask &= (grid == 0)
        else:
            inflate_mask &= (grid <= 0)

        out[inflate_mask] = 100
        out[occupied] = 100

        return out


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = InflationPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'inflation_publisher error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()