import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from custom_msg.msg import Imu
from .clustercontroller import Cluster
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String

from .my_ros_module import PubSubManager

class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_controller')
        self.traj_publichers
        self.cluster = Cluster()
        self.create_subscription(Imu, "/imu/eulerangle", self.Imu_callback, 10)
        

       
    #     timer_period = 5  # seconds
    #     self.timer = self.create_timer(timer_period, self.listen_for_publishers)

    # def listen_for_publishers(self):
    #     topic_names_and_types = self.get_topic_names_and_types()
    #     for topic, types in topic_names_and_types:
    #         if 'robot' in topic and '/imu/eulerAngle' in topic:
    #             self.create_subscription(Float32MultiArray, topic, self.listener_callback, 10)
    #             self.get_logger().info(f'Subscribed to: {topic}')

    def Imu_callback(self, msg, msg_info):
        self.get_logger().info(f'Received: "{msg.heading}" from robot: "{msg.robotid}"')
        cluster.updateRobotHeading(msg.robotid, msg.heading)

    def calculateLinearControl():
        self.Fm = self.cdddes + self.Kv*()
def main(args=None):
    rclpy.init(args=args)
    node = ClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
