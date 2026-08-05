import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid


WHITESPACE = set(b' \t\n\r\x0b\x0c')


def read_pgm(path: str) -> np.ndarray:
    """
    Minimal PGM reader for P2 ASCII and P5 binary.

    Returns numpy array shape=(height, width), dtype uint8.
    """
    with open(path, 'rb') as f:
        raw = f.read()

    idx = 0

    def skip_whitespace_and_comments():
        nonlocal idx
        while idx < len(raw):
            c = raw[idx]
            if c in WHITESPACE:
                idx += 1
            elif c == ord('#'):
                while idx < len(raw) and raw[idx] != ord('\n'):
                    idx += 1
            else:
                break

    def read_token() -> bytes:
        nonlocal idx
        skip_whitespace_and_comments()
        start = idx
        while idx < len(raw) and raw[idx] not in WHITESPACE:
            idx += 1
        return raw[start:idx]

    magic = read_token()
    width = int(read_token())
    height = int(read_token())
    maxval = int(read_token())

    # After maxval PGM expects one whitespace before binary data.
    if idx < len(raw) and raw[idx] in WHITESPACE:
        idx += 1

    if magic == b'P5':
        if maxval <= 255:
            expected = width * height
            arr = np.frombuffer(raw[idx:idx + expected], dtype=np.uint8)
        else:
            expected = width * height * 2
            arr = np.frombuffer(raw[idx:idx + expected], dtype=np.dtype('>u2'))
            arr = (arr.astype(np.float32) * 255.0 / float(maxval)).astype(np.uint8)

        if arr.size < width * height:
            raise ValueError('PGM binary data is shorter than expected.')

        arr = arr[:width * height].reshape((height, width))

    elif magic == b'P2':
        tokens = raw[idx:].split()
        if len(tokens) < width * height:
            raise ValueError('PGM ASCII data is shorter than expected.')

        arr = np.array([int(t) for t in tokens[:width * height]], dtype=np.uint8)

        if maxval != 255:
            arr = (arr.astype(np.float32) * 255.0 / float(maxval)).astype(np.uint8)

        arr = arr.reshape((height, width))

    else:
        raise ValueError(f'Unsupported PGM magic: {magic}')

    return arr


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 0.0)

        map_yaml = self.get_parameter('map_yaml').value
        if not map_yaml:
            raise RuntimeError(
                "Parameter 'map_yaml' is required. "
                "Set it to absolute path of map.yaml"
            )

        self.grid = self.load_map(map_yaml)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        topic = self.get_parameter('map_topic').value
        self.pub = self.create_publisher(OccupancyGrid, topic, qos)

        publish_rate = self.get_parameter('publish_rate').value
        if publish_rate > 0.0:
            self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)

        self.publish_map()

        self.get_logger().info(
            f'Map published on topic {topic} from {map_yaml}'
        )

    def load_map(self, yaml_path: str) -> OccupancyGrid:
        yaml_path = os.path.abspath(yaml_path)

        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        image = cfg.get('image', '')
        if not image:
            raise RuntimeError("map.yaml does not contain 'image' field.")

        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(yaml_path), image)

        img = read_pgm(image)

        resolution = float(cfg.get('resolution', 0.05))
        origin = cfg.get('origin', [0.0, 0.0, 0.0])
        occupied_thresh = float(cfg.get('occupied_thresh', 0.65))
        free_thresh = float(cfg.get('free_thresh', 0.25))
        negate = bool(cfg.get('negate', 0))
        mode = cfg.get('mode', 'trinary')

        if mode != 'trinary':
            self.get_logger().warning(
                f"Map mode '{mode}' is not fully supported by this simple publisher. "
                "Using trinary-like conversion."
            )

        # Typical ROS map convention:
        # black = occupied, white = free, gray = unknown.
        occupancy = (255.0 - img.astype(np.float32)) / 255.0

        if negate:
            occupancy = 1.0 - occupancy

        data = np.full(img.shape, -1, dtype=np.int8)

        # Free cells
        data[occupancy <= free_thresh] = 0

        # Occupied cells
        data[occupancy >= occupied_thresh] = 100

        # Image row 0 is top, OccupancyGrid row 0 is bottom.
        data = np.flipud(data).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = resolution
        msg.info.width = int(img.shape[1])
        msg.info.height = int(img.shape[0])

        origin_x = float(origin[0]) if len(origin) > 0 else 0.0
        origin_y = float(origin[1]) if len(origin) > 1 else 0.0
        origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0

        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation = yaw_to_quaternion(origin_yaw)

        msg.data = data.ravel().tolist()

        return msg

    def publish_map(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MapPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'map_publisher error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()