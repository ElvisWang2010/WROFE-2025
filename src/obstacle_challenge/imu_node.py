import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import math
import ros_robot_controller_sdk as rrc

def radians_to_degrees(rad):
    return math.degrees(rad) % 360

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/rpy/filtered',
            self.imu_callback,
            10
        )
        board = rrc.Board()
        self.gyro_pub = self.create_publisher(Float32, '/imu_angle', 10)
        self.prev_angle = None

    def imu_callback(self, msg):
        current_angle_deg = radians_to_degrees(msg.vector.z)
        yaw_msg = Float32()
        yaw_msg.data = current_angle_deg
        self.gyro_pub.publish(yaw_msg)

        self.get_logger().info(f"Yaw angle: {current_angle_deg:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
