import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class JetsonCamNode(Node):
    def __init__(self, sensor_id, topic_name):
        super().__init__(f'jetson_cam_{sensor_id}')
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()
        
        self.gst_pipeline = (
            f"v4l2src device=/dev/video{sensor_id} ! "
            "video/x-raw, format=RG10, width=1280, height=720, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=RGB ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=1 max-buffers=2"
        )
        
        self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error(f"Не удалось открыть камеру {sensor_id}. Проверьте GStreamer и устройства.")
            return
            
        self.timer = self.create_timer(0.033, self.timer_callback) # ~30 FPS
        self.get_logger().info(f"Камера {sensor_id} запущена на топике {topic_name}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_optical_frame"
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Не удалось получить кадр")

def main(args=None):
    rclpy.init(args=args)
    node0 = JetsonCamNode(sensor_id=0, topic_name="/camera_0/image_raw")
    node1 = JetsonCamNode(sensor_id=1, topic_name="/camera_1/image_raw")
    
    try:
        rclpy.spin(node0)
        rclpy.spin(node1)
    except KeyboardInterrupt:
        pass
    finally:
        node0.cap.release()
        node1.cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()