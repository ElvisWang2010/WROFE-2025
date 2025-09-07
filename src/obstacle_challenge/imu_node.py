#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Bool
import math

def radians_to_degrees(rad):
    return math.degrees(rad) % 360

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.subscription = self.create_subscription(Vector3Stamped,'/imu/rpy/filtered', self.imu_callback, 10)
        self.lap_done_pub = self.create_publisher(Bool, '/lap_status', 10)
        self.gyro_pub = self.create_publisher(Float32, '/imu_angle', 10)
        self.create_subscription(Bool, '/state', self.stop_callback, 10)
        
        self.initial_angle = None
        self.prev_normalized_angle = None
        self.total_rotation = 0.0
        self.lap_count = 0
        self.max_laps = 3
        self.laps_done = False

    def stop_callback(self, msg):
        if msg.data == "stop":
            self.get_logger().info("Received stop command, shutting down")
            rclpy.shutdown()

    def imu_callback(self, msg):
        current_angle = radians_to_degrees(msg.vector.z)

        # Set initial angle on first message
        if self.initial_angle is None:
            self.initial_angle = current_angle
            self.prev_normalized_angle = 0.0
            return
        # calibrate angle
        normalized_angle = (current_angle - self.initial_angle) % 360
        if normalized_angle < 0:
            normalized_angle += 360

        delta = normalized_angle - self.prev_normalized_angle
        
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self.total_rotation += abs(delta)
        self.prev_normalized_angle = normalized_angle

        if not self.laps_done and self.total_rotation >= 360.0:
            self.lap_count += 1
            self.total_rotation -= 360.0

            self.get_logger().info(f"Lap {self.lap_count} completed! Total rotation: {self.total_rotation + 360.0:.1f}")

            if self.lap_count >= self.max_laps:
                msg_out = Bool()
                msg_out.data = True
                self.lap_done_pub.publish(msg_out)
                self.get_logger().info("3 laps done!")
                self.laps_done = True
    
        # Publish fixed angle
        yaw_msg = Float32()
        yaw_msg.data = normalized_angle
        self.gyro_pub.publish(yaw_msg)
        
        # Debug logging
        self.get_logger().info(f"Norm Angle: {normalized_angle:.1f}, Delta: {delta:.1f}, Total: {self.total_rotation:.1f}, Lap: {self.lap_count}")

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
