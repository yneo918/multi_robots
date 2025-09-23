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

FREQ = 10 # Frequency to publish velocity commands
JOY_FREQ = FREQ
KV = 0.5  # Gain for computed velocity commands
MAX_DB = -10.0  # Max expected RSSI value in dBm
MIN_DB = -105.0  # Min expected RSSI value in dBm
DESIRED_DB = -55.0  # Desired RSSI value in dBm for cross-track controller

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
        self.robot_id_list = self.get_parameter('robot_id_list').value # List of robot IDs to include in gradient calculation

        self.pubsub = PubSubManager(self)
        self.set_pubsub()

        self.gradient = ScalarGradient(num_robots=len(self.robot_id_list))
        self.sim_gradient = ScalarGradient(num_robots=len(self.robot_id_list))

        self.future = [None] * len(self.robot_id_list)  # Store futures for each robot

        timer_period = 1 / FREQ 
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)
        self.output = "actual" 
        self.enable = False

        self.z = DESIRED_DB  # Desired RSSI value for the robot to navigate towards, in dBm

        self.cli = self.create_client(GetRxPower, '/get_rx_power') #service to get the RSSI values in simulation

    def set_pubsub(self):
        """Set up publishers and subscribers."""
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
                Float64,
                f'/{robot_id}/rssi',
                lambda msg, robot_id=robot_id: self.actual_rssi(msg, robot_id),
                5)
            self.pubsub.create_subscription(
                Float64,
                f'/sim/{robot_id}/rssi',
                lambda msg, robot_id=robot_id: self.sim_rssi(msg, robot_id),
                5)
        self.get_logger().info(f"Listening for robots: {self.robot_id_list}")

    def actual_pose(self, msg, robot_id):
        """Update the robot's position in (x, y, z) where z is height in scalar field"""
        robot_pose = [msg.x , msg.y]
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][0:2] = robot_pose

    def sim_pose(self, msg, robot_id):
        """Update the robot's position in (x, y, z) where z is height in scalar field"""
        robot_pose = [msg.x , msg.y]
        self.sim_gradient.robot_positions[self.robot_id_list.index(robot_id)][0:2] = robot_pose

    def actual_rssi(self, msg, robot_id):
        """Update the robot's height in (x, y, z) where z is height in scalar field"""
        self.gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = self.normalize_db(msg.data, db_min=MAX_DB, db_max=MIN_DB)  # Set min and max based on expected RSSI range

    def sim_rssi(self, msg, robot_id): 
        """Update the robot's height in (x, y, z) where z is height in scalar field"""
        self.sim_gradient.robot_positions[self.robot_id_list.index(robot_id)][2] = self.normalize_db(msg.data, db_min=-70.0, db_max=-33.0)  # Normalize the RSSI value
        #self.get_logger().info(f"Simulated RSSI for {robot_id}: {self.normalize_db(msg.data)}") #values range from -33 to -70

    def normalize_db(self, db_val, db_min=-105.0, db_max=-65.0):
        """Normalize dBm value to a 0-1 scale which greatly improves gradient calculation."""
        db_val_clipped = max(min(db_val, db_max), db_min)  # Clamp to expected range
        norm = (db_val_clipped - db_min) / (db_max - db_min)
        return norm

    def update_adaptive_mode(self, msg):
        """Update the control mode based on incoming message."""
        for mode in ControlMode:
            if msg.data == mode.value:
                temp = self.gradient.mode
                self.gradient.mode = mode
                self.sim_gradient.mode = mode
                if temp != mode:
                    self.get_logger().info(f"Adaptive mode changed from {temp.value} to {mode.value}")

    def _mode_callback(self, msg: String):
        """Enable or disable adaptive navigation based on mode message."""
        if msg.data == "ADPTV_NAV_M":
            self.enable = True
        else:
            self.enable = False

    def publish_velocities(self, output):
        """Compute gradient and publish velocity commands."""
        _msg = Twist()
        bearing = self.gradient.get_velocity(zdes=self.normalize_db(self.z)) if output == 'actual' else self.sim_gradient.get_velocity(zdes=self.normalize_db(self.z))
        if bearing is None:
            self.get_logger().warn("No bearing calculated, skipping publish.")
            return
        else:
            self.get_logger().info(f"Publishing bearing: {bearing} for output: {output}")
        _msg.linear.x = math.cos(bearing) * KV
        _msg.linear.y = math.sin(bearing) * KV
        self.pubsub.publish('/ctrl/cmd_vel', _msg) #publish velocity command to cluster

    def publish_velocities_manager(self):
        """ Manage which velocities to publish based on the current output mode."""
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
        """Set output mode based on joystick input."""
        temp = self.output
        if not msg.data:
            self.output = "sim"
            count = 0
            while count < 10 and not self.cli.wait_for_service(timeout_sec=1.0):
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