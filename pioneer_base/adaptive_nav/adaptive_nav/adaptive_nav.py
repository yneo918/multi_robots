import rclpy
import math
import numpy as np
import time
from rclpy.node import Node
from .ScalarGradient import ScalarGradient
from std_msgs.msg import Bool, Int16, String, Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from teleop_core.my_ros_module import PubSubManager

FREQ = 10
JOY_FREQ = FREQ

class ANNode(Node):
    def __init__(self):
        super().__init__('cluster_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p2", "p3", "p4"]),
                ('cluster_size', 3), 
            ]
        )
        params = self._parameters
        robot_id_list = self.get_parameter('robot_id_list').value
        self.set_robot_id_list(robot_id_list)

        self.pubsub = PubSubManager(self)
        self.gradient = ScalarGradient(num_robots=len(self.robot_id_list))
        self.sim_gradient = ScalarGradient(num_robots=len(self.robot_id_list))

        timer_period = 1 / FREQ  # set frequency to publish velocity commands
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)

    def set_pubsub(self):
        self.pubsub.create_publisher(Twist, '/joy/cmd_vel', 5) #publish to cluster
        for robot_id in self.robot_id_list:
            self.pubsub.create_subscription(
                Pose2D,
                f'/{robot_id}/pose2D',
                lambda msg, robot_id=robot_id: self.actual_pose(msg, robot_id),
                5)
            self.pubsub.create_subscription(
                Pose2D,
                f'/sim/{robot_id}/pose2D',
                lambda msg, robot_id=robot_id: self.sim_pose(msg, robot_id),
                5)
            self.pubsub.create_subscription(
                Int16,
                f'/{robot_id}/rssi',
                lambda msg, robot_id=robot_id: self.actual_rssi(msg, robot_id),
                5)
            self.pubsub.create_subscription(
                Int16,
                f'/sim/{robot_id}/rssi',
                lambda msg, robot_id=robot_id: self.sim_rssi(msg, robot_id),
                5)
        self.get_logger().info(f"Listening for robots: {self.robot_id_list}")

    def actual_pose(self, msg, robot_id):
        robot_pose = [msg.x , msg.y]
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][0:2] = robot_pose

    def sim_pose(self, msg, robot_id):
        robot_pose = [msg.x , msg.y]
        self.sim_gradient.robot_positions[self.robot_id_list.index(robot_id)][0:2] = robot_pose

    def actual_rssi(self, msg, robot_id):
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = msg.data

    def sim_rssi(self, msg, robot_id):
        self.sim_gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = msg.data

    def publish_velocities(self, output):
        _msg = Twist()
        _msg = self.gradient.get_velocity() if output == 'actual' else self.sim_gradient.get_velocity()
        self.pubsub.publish('/joy/cmd_vel', _msg)

    def publish_velocities_manager(self):
        if self.both:
            if self.actual_configured:
                self.publish_velocities("actual")
            if self.sim_configured:
                self.publish_velocities("sim")
        if self.output == "actual" and self.actual_configured:
            self.publish_velocities("actual")
        elif self.output == "sim" and self.sim_configured:
            self.publish_velocities("sim")
        
def main(args=None):
    rclpy.init(args=args)
    node = ANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()