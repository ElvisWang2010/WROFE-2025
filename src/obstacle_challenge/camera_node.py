import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2
import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.subscription = self.create_subscription(Bool, '/stop_signal', self.stop_callback, 10)
        self.bridge = CvBridge()
        self.picam2 = Picamera2()
        self.picam2.start()
        time.sleep(1)

        self.timer = self.create_timer(0.1, self.publish_frame) 

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info("Lap completed. Stopping camera node.")
            rclpy.shutdown()

    def publish_frame(self):
        frame = self.picam2.capture_array()
        if frame is not None:
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

