import rclpy
import math
import numpy as np
import time
from rclpy.node import Node
from .ScalarGradient import ScalarGradient, ControlMode
from std_msgs.msg import Bool, Int16, String, Float32MultiArray, Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from teleop_core.my_ros_module import PubSubManager
from rf_sim_interfaces.srv import GetRxPower

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

        self.future = [None] * len(self.robot_id_list)  # Store futures for each robot

        timer_period = 1 / FREQ  # set frequency to publish velocity commands
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)
        self.output = "actual" 
        self.z = -10.0  

        self.cli = self.create_client(GetRxPower, 'get_rx_power') #service to get the RSSI values
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service get_rx_power...')
    def set_pubsub(self):
        self.pubsub.create_subscription(Bool, '/joy/hardware', self.hw_sim_callback, 1)
        self.pubsub.create_publisher(Twist, '/joy/cmd_vel', 5) #publish to cluster
        self.pubsub.create_subscription(String, '/modeA', self.update_adaptive_mode, 1)

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
        self.get_logger().info(f"Simulated RSSI for {robot_id}: {msg.data}")

    def update_adaptive_mode(self, msg):
        for mode in ControlMode:
            if msg.data == mode.value:
                temp = self.gradient.mode
                self.gradient.mode = mode
                self.sim_gradient.mode = mode
                if temp != mode:
                    self.get_logger().info(f"Adaptive mode changed from {temp.value} to {mode.value}")

    def publish_velocities(self, output):
        _msg = Twist()
        velocity = self.gradient.get_velocity() if output == 'actual' else self.sim_gradient.get_velocity()
       # if velocity is None:
        #    self.get_logger().warn("No velocity calculated, skipping publish.")
         #   return
        #else:
         #   self.get_logger().info(f"Publishing velocity: {velocity} for output: {output}")
        _msg.linear.x = math.cos(velocity)
        _msg.linear.y = math.sin(velocity)
        self.pubsub.publish('/joy/cmd_vel', _msg)

    def publish_velocities_manager(self):
        if self.output == "actual":
            self.publish_velocities("actual")
        elif self.output == "sim":
            for i in range(len(self.sim_gradient.robot_positions)):
                if self.sim_gradient.robot_positions[i][0] == None or self.sim_gradient.robot_positions[i][1] == None:
                    self.get_logger().warn(f"Robot {self.robot_id_list[i]} has no position data, skipping simulation velocity calculation.")
                    return
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