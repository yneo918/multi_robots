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
        super().__init__('adaptive_navigator')
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

        #self.gradient.mode = ControlMode.MAX
        #self.sim_gradient.mode = ControlMode.MAX

        self.future = [None] * len(self.robot_id_list)  # Store futures for each robot

        timer_period = 1 / FREQ  # set frequency to publish velocity commands
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)
        self.output = "actual" 
        self.enable = False

        self.z = -65.0  # Desired RSSI value for the robot to navigate towards, in dBm

        self.cli = self.create_client(GetRxPower, 'get_rx_power') #service to get the RSSI values
    def set_pubsub(self):
        self.pubsub.create_subscription(Bool, '/joy/hardware', self.hw_sim_callback, 1)
        self.pubsub.create_publisher(Twist, '/ctrl/cmd_vel', 5) #publish to cluster
        self.pubsub.create_subscription(String, '/modeA', self.update_adaptive_mode, 1)
        self.pubsub.create_subscription(String, '/modeC', self._mode_callback, 1)
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
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = self.normalize_db(msg.data)  # Normalize the RSSI value
        #self.gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = 10 ** (msg.data/ 10) # Convert dBm to power to linearize

    def sim_rssi(self, msg, robot_id): 
        self.sim_gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = self.normalize_db(msg.data)  # Normalize the RSSI value
        #self.sim_gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = 10 ** (msg.data/ 10) # Convert dBm to power to linearize
        #self.get_logger().info(f"Simulated RSSI for {robot_id}: {self.normalize_db(msg.data)}") #values range from -33 to -70

    def normalize_db(self, db_val, db_min=-90.0, db_max=-33.0):
        db_val_clipped = max(min(db_val, db_max), db_min)  # Clamp to expected range
        norm = (db_val_clipped - db_min) / (db_max - db_min)
        return norm

    def update_adaptive_mode(self, msg):
        for mode in ControlMode:
            if msg.data == mode.value:
                temp = self.gradient.mode
                self.gradient.mode = mode
                self.sim_gradient.mode = mode
                if temp != mode:
                    self.get_logger().info(f"Adaptive mode changed from {temp.value} to {mode.value}")

    def _mode_callback(self, msg: String):
        if msg.data == "ADPTV_NAV_M":
            self.enable = True
        else:
            self.enable = False

    def publish_velocities(self, output):
        _msg = Twist()
        bearing = self.gradient.get_velocity(zdes=self.normalize_db(self.z)) if output == 'actual' else self.sim_gradient.get_velocity(zdes=self.normalize_db(self.z))
        if bearing is None:
            self.get_logger().warn("No bearing calculated, skipping publish.")
            return
        else:
            self.get_logger().info(f"Publishing bearing: {bearing} for output: {output}")
        _msg.linear.x = math.cos(bearing) * 0.5
        _msg.linear.y = math.sin(bearing) * 0.5
        self.pubsub.publish('/ctrl/cmd_vel', _msg)

    def publish_velocities_manager(self):
        if self.enable:
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
            count = 0
            while count < 10 or not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for service get_rx_power...')
                count  += 1
            if(count >=10):
                self.get_logger().info('Unable to connect to sim rssi changing mode to idle')
            self.output = "idle"
        else:
            self.output = "actual"
        if temp != self.output:
            self.get_logger().info(f"Changed output to {self.output} for adaptive navigation.")

def main(args=None):
    rclpy.init(args=args)
    node = ANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()