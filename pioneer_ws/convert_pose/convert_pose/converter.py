import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

import datetime
import os


class PoseConverter(Node):
    def __init__(self, n_rover=6):
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_pose_converter')
        # Temporary values / Garage
        self.ref_lat = -121.94158
        self.ref_lon = 37.35232

        self.create_subscription(
            NavSatFix,
            f"/{self.robot_id}/gps1",
            self.gps_callback,
            1)
        self.create_subscription(
            Quaternion,
            f"/{self.robot_id}/imu/quaternion",
            self.quaternion_callback
            1)
        self.create_subscription(
            Float32MultiArray,
            f"/{self.robot_id}/imu/eulerAngle",
            self.euler_callback,
            1)
        self.pose_publisher = self.create_publisher(
            Pose2D,
            f'/{self.robot_id}/pose2D',
            5)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.gps_status = msg.status.status
    
    def quaternion_callback(self, msg):
        self.quaternion = msg
    
    def euler_callback(self, msg):
        self.euler_x = msg.data[0]
        self.euler_y = msg.data[1]
        self.euler_z = msg.data[2]
    
    def timer_callback(self):
        msg = Pose2D()
        msg.x = self.lat
        msg.y = self.lon
        msg.theta = self.euler_x
        self.pose_publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.x}, {msg.y}, {msg.theta}")

def main(args=None):
    rclpy.init(args=args)
    converter = PoseConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
