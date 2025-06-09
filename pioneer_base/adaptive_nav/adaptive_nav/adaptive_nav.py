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

"""
The AN Node manages collects all relevant data from active robots and sends velocity command from the ScalarGradient class.
Will pulbish the values for sim or actual robots based on the which is active from joystick.
@params:
    robot_id_list: list of robot ids to listen for
"""

FREQ = 10
JOY_FREQ = FREQ

class ANNode(Node):
    def __init__(self):
        super().__init__('cluster_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p2", "p3", "p4"]),
            ]
        )
        params = self._parameters
        self.robot_id_list = self.get_parameter('robot_id_list').value

        self.pubsub = PubSubManager(self)
        self.set_pubsub()

        self.gradient = ScalarGradient(num_robots=len(self.robot_id_list))
        self.sim_gradient = ScalarGradient(num_robots=len(self.robot_id_list))

        timer_period = 1 / FREQ  # set frequency to publish velocity commands
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)
        self.output = "actual" 

    def set_pubsub(self):
        self.pubsub.create_subscription(Bool, '/joy/hardware', self.hw_sim_callback, 1)
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
        velocity = self.gradient.get_velocity() if output == 'actual' else self.sim_gradient.get_velocity()
        if velocity is None or not len(velocity) == 2:
            self.get_logger().warn("No velocity calculated, skipping publish.")
            return
        _msg.linear.x = velocity[0]
        _msg.linear.y = velocity[1]
        self.pubsub.publish('/joy/cmd_vel', _msg)

    def publish_velocities_manager(self):
        if self.output == "actual":
            self.publish_velocities("actual")
        elif self.output == "sim":
            self.publish_velocities("sim")
        
    def hw_sim_callback(self, msg):
        temp = self.output
        if not msg.data:
            self.output = "sim"
        else:
            self.output = "actual"
        if temp != self.output:
            self.get_logger().info(f"Changed output to {self.output}")
    
def main(args=None):
    rclpy.init(args=args)
    node = ANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()