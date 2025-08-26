import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Bool
import math
from obstacle_challenge import ros_robot_controller_sdk as rrc

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

        self.lap_done_pub = self.create_publisher(Bool, '/imu_status', 10)
        self.gyro_pub = self.create_publisher(Float32, '/imu_angle', 10)
        self.create_subscription(Bool, '/stop_signal', self.stop_callback, 10)
        self.prev_angle = None
        self.total_rotation = 0.0
        self.lap_count = 0
        self.max_laps = 3
        self.laps_done = False
        self.error = 10

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info("Lap completed. Stopping imu node.")
            rclpy.shutdown()

    def imu_callback(self, msg):
        current_angle = radians_to_degrees(msg.vector.z)

        if self.prev_angle is None:
            self.prev_angle = current_angle
            return

        delta = current_angle - self.prev_angle
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self.total_rotation += delta
        self.prev_angle = current_angle

        if abs(self.total_rotation) >= (360 - self.error) and not self.laps_done:
            self.lap_count += 1
            self.total_rotation = 0.0

            self.get_logger().info(f"Lap completed! Total laps: {self.lap_count}")

            if self.lap_count >= self.max_laps and not self.laps_done:
                msg_out = Bool()
                msg_out.data = True
                self.lap_done_pub.publish(msg_out)
                self.get_logger().info("3 laps done!")
                self.laps_done = True
    
        yaw_msg = Float32()
        yaw_msg.data = current_angle
        self.gyro_pub.publish(yaw_msg)
        self.get_logger().info(f"current angle: {yaw_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
