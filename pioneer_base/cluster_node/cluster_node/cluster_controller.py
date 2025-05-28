import rclpy
import math
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from .Cluster import Cluster, ClusterConfig
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from pioneer_interfaces.msg import ClusterInfo

from teleop_core.my_ros_module import PubSubManager
"""
The Cluster Node manages communication between the commanded cluster velocity and the calculated velocities for each robot in the cluster.
@params:
    robot_id_list: list of robot ids to listen for
    cluster_size: number of robots to form cluster with
On startup the node will listen for all active robots in robot_id_list. Once the mode is switched to cluster control, 
if there are enough robots to form a cluster, it initializes a Cluster object and map the robot ids to the cluster.
"""

FREQ = 10
JOY_FREQ = FREQ
KP_GAIN = 1.0
KV_GAIN = 1.0
EPSILON = 0.1
MAX_VEL = 0.5
ROVER_DOF = 3 # (x, y, theta)

class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p1", "p2", "p3", "p4", "p5"]),
                ('cluster_size', 5), 
                ('cluster_params', [3, 3, 3, 3, 0, 0, 0]), #cluster parameters
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")
        
        #Parse robot ID list
        robot_id_list = self.get_parameter('robot_id_list').value
        self.set_robot_id_list(robot_id_list)
        self.get_logger().info(f"ROVER: {self.robot_id_list} N: {self.n_rover}")

        self.cluster_params = self.get_parameter('cluster_params').value
        self.get_logger().info(f"Cluster parameters: {self.cluster_params}")

        self.initialize_cluster()
        
        self.pubsub = PubSubManager(self)
        self.set_pubsub()

        self.cluster_enable = False
        timer_period = 1 / FREQ  # set frequency to publish velocity commands
        self.vel_timer = self.create_timer(timer_period, self.publish_velocities_manager)
    
    def set_robot_id_list(self, robot_id_list):
        self.robot_id_list = []
        self.actual_robot_status = {} # (x, y, theta, vel_x, vel_y, vel_theta, z)
        self.sim_robot_status = {} # (x, y, theta, vel_x, vel_y, vel_theta, z)
        if isinstance(robot_id_list, list):
            self.n_rover = len(robot_id_list)
            self.robot_id_list = robot_id_list
            for robot_id in self.robot_id_list:
                self.actual_robot_status[robot_id] = [None, None, None, None, None, None, None] # Initialize rover status
                self.sim_robot_status[robot_id] = [None, None, None, None, None, None, None] # Initialize sim rover status
        else:
            raise ValueError("robot_id_list must be a list of strings")
    
    def initialize_cluster(self):
        self.both = False #flag to check if both actual and simulation robots are present
        self.actual_configured = False
        self.sim_configured = False

        #All data on actual robots
        self.cluster = None #cluster object to be initialized when cluster is formed
        self.cluster_size = self.get_parameter('cluster_size').value
        self.cluster_robots = [] #list of active robots in cluster
        self.registered_robots = [] #list of active robots in cluster
        self.c_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(self.cluster_params, (len(self.cluster_params), 1))
        self.cdot_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.r = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.rdot = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.time = None
        self.cluster = Cluster(num_robots = self.cluster_size, cluster_type=ClusterConfig.TRILEAD, KPgains=[KP_GAIN]*(self.cluster_size*ROVER_DOF), KVgains=[KV_GAIN]*(self.cluster_size*ROVER_DOF))

        #All data on simulation robots
        self.sim_cluster_robots = [] #list of active robots in cluster
        self.sim_registered_robots = [] #list of active robots in cluster
        self.sim_c_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.sim_c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(self.cluster_params, (len(self.cluster_params), 1))
        self.sim_cdot_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.sim_r = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.sim_rdot = np.zeros((self.cluster_size*ROVER_DOF, 1))
        self.sim_time = None

        #Set up for both actual and simulation robots
        self.output = "actual" #switch between outputing velocity to simulation or actual robots
        self.mode = "INIT" #switch between manual or cluster control
        self.joy_timestamp = None
        self.enable = False
        self.get_logger().info(f"Cluster initialized with size: {self.cluster_size} and sim size: {self.cluster_size}")
    
    def set_pubsub(self):
        self.pubsub.create_subscription(Bool, '/joy/hardware', self.hw_sim_callback, 1)
        self.pubsub.create_subscription(String, '/modeC', self.mode_callback, 1)
        self.pubsub.create_subscription(Twist, '/joy/cmd_vel', self.joycmd_callback, 5)
        self.pubsub.create_subscription(Bool, '/joy/enable', self.enable_callback, 5)
        self.pubsub.create_subscription(Float32MultiArray, '/cluster_params', self.cluster_params_callback, 5)
        self.pubsub.create_subscription(Float32MultiArray, '/cluster_desired', self.cluster_desired_callback, 5)
        
        self.pubsub.create_publisher(ClusterInfo, '/cluster_info', 5)
        self.pubsub.create_publisher(ClusterInfo, '/sim/cluster_info', 5)

        for robot_id in self.robot_id_list:
            self.pubsub.create_subscription(
                Pose2D,
                f'/{robot_id}/pose2D',
                lambda msg, robot_id=robot_id: self.actual_callback(msg, robot_id),
                5)
            self.pubsub.create_subscription(
                Pose2D,
                f'/sim/{robot_id}/pose2D',
                lambda msg, robot_id=robot_id: self.sim_callback(msg, robot_id),
                5)
            self.pubsub.create_publisher(
                Pose2D,
                f'/{robot_id}/desiredPose2D',
                5)
            self.pubsub.create_publisher(
                Twist, 
                f'/{robot_id}/cmd_vel', 5)
        self.get_logger().info(f"Listening for robots: {self.robot_id_list}")

    #after listening for nearby robots assign them to cluster 
    def assign_robots(self, actual_sim):
        if actual_sim == "actual":
            self.cluster_robots.sort() #sort the robot ids
            #actual robots
            self.registered_robots = self.cluster_robots[0:self.cluster_size] #trim extra robots
            # change configure of cluster here
            self.get_logger().info(f"Cluster status: {self.cluster.c_des}")
            self.get_logger().info(f"Formed cluster with robots: {self.registered_robots} from list of possible: {self.robot_id_list}")
            self.get_logger().info(f"Desired robot position: {self.cluster.get_desired_position(self.c_des)}")
            # change configure of cluster here
            self.actual_configured = True
        elif actual_sim == "sim":
            self.sim_cluster_robots.sort() 
            #simulation robots
            self.sim_registered_robots = self.sim_cluster_robots[0:self.cluster_size] #trim extra robots
            self.sim_configured = True
            
        self.get_logger().info(f"Cluster_robots: {self.registered_robots} Sim_cluster_robots: {self.sim_registered_robots}")
        
    #Choose from modes ["NEU_M", "JOY_M", "NAV_M"]
    def mode_callback(self, msg):
        self.mode = msg.data
        if self.mode == "JOY_M" or self.mode == "NEU_M":
            self.cluster_enable = False
        elif self.mode == "NAV_M":
            self.cluster_enable = True

    def hw_sim_callback(self, msg):
        temp = self.output
        if not msg.data:
            self.output = "sim"
        else:
            self.output = "actual"
        if temp != self.output:
            self.get_logger().info(f"Changed output to {self.output}")
    
    def enable_callback(self, msg):
        self.enable = msg.data

    def angular_callback(self, msg, i):
        self.check_robot_id(i, "actual")
        cluster_index = self.map_robot_id(i, "actual")
        if cluster_index is None:
            return
        self.r[cluster_index*ROVER_DOF+2, 0] = msg.data[-1] #update robot heading in array

    def actual_callback(self, msg, robot_id):
        robot_pose = [msg.x , msg.y, msg.theta]
        past_robot_status = self.actual_robot_status[robot_id]
        past_robot_pose = past_robot_status[0:3]
        robot_vel = [ (msg.x - past_robot_pose[0]) * FREQ,
                       (msg.y - past_robot_pose[1]) * FREQ,
                       (msg.theta - past_robot_pose[2]) * FREQ]
        robot_status = robot_pose + robot_vel + past_robot_status[6:] #update robot status
        self.actual_robot_status[robot_id] = robot_status

    def sim_callback(self, msg, robot_id):
        self.check_robot_id(i, "sim")
        if not self.sim_configured:
            return
        cluster_index = self.map_robot_id(robot_id, "sim")
        if cluster_index is None:
            return
        if self.output == "sim" or self.both:
            robot_pose = [msg.x , msg.y, msg.theta]
            self.sim_r[cluster_index*ROVER_DOF:((cluster_index+1)*ROVER_DOF), 0] = robot_pose #update robot position in array 
            _desired = self.cluster.get_desired_position(self.sim_c_des)
            if _desired is None:
                return
            #self.get_logger().info(f"Sim robot positions {self.sim_r} Desired sim robot position: {_desired}")
            _pose = Pose2D()
            _pose.x, _pose.y, _pose.theta = _desired[cluster_index*ROVER_DOF+0, 0], _desired[cluster_index*ROVER_DOF+1, 0], _desired[cluster_index*ROVER_DOF+2, 0]
            self.pubsub.publish(f'/{self.robot_id_list[self.sim_cluster_robots[cluster_index]]}/desiredPose2D', _pose)
                
    #Velocity command from the joystick to be sent to the cluster
    def joycmd_callback(self, msg):
        freq = 1 / (time.time() - self.joy_timestamp ) if self.joy_timestamp is not None else JOY_FREQ
        self.joy_timestamp = time.time()
        if self.mode == "NAV_M":
            v_x = -msg.linear.x
            v_y = msg.linear.y
            v_r = -msg.angular.z * 0.3
            if self.output == "actual" or self.both:
                self.c_des[0, 0] += v_x / freq
                self.c_des[1, 0] += v_y / freq
                self.c_des[2, 0] += v_r / freq
                self.c_des[2, 0] = self.wrap_to_pi(self.c_des[2, 0])
            if self.output == "sim" or self.both:
                self.sim_c_des[0, 0] += v_x / freq
                self.sim_c_des[1, 0] += v_y / freq
                self.sim_c_des[2, 0] += v_r / freq
                self.sim_c_des[2, 0] = self.wrap_to_pi(self.sim_c_des[2, 0])      
            self.get_logger().info(f"Cluster desired position: {self.c_des.flatten()}")
            self.get_logger().info(f"Sim cluster desired position: {self.sim_c_des.flatten()}")          

    #Set cluster parameters from the cluster_params topic
    def cluster_params_callback(self, msg):
        self.get_logger().info(f"Received cluster parameters: {msg.data}")
        if msg.data[0] < 0.0 and msg.data[1] < 0.0 and msg.data[2] < 0.0:
            self.get_logger().info("Resetting cluster parameters")
            if self.both:
                if self.actual_configured:
                    self.c_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
                    self.c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(self.cluster_params, (len(self.cluster_params), 1))
                if self.sim_configured:
                    self.sim_c_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
                    self.sim_c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(self.cluster_params, (len(self.cluster_params), 1))
            else:
                if self.output == "actual" and self.actual_configured:
                    self.c_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
                    self.c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(self.cluster_params, (len(self.cluster_params), 1))
                elif self.output == "sim" and self.sim_configured:
                    self.sim_c_des = np.zeros((self.cluster_size*ROVER_DOF, 1))
                    self.sim_c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(self.cluster_params, (len(self.cluster_params), 1))
            return
        if self.both:
            if self.actual_configured:
                self.c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(msg.data, (len(msg.data), 1))
            if self.sim_configured:
                self.sim_c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(msg.data, (len(msg.data), 1))
        else:
            if self.output == "actual" and self.actual_configured:
                self.c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(msg.data, (len(msg.data), 1))
            elif self.output == "sim" and self.sim_configured:
                self.sim_c_des[self.n_rover*ROVER_DOF - len(self.cluster_params):self.n_rover*ROVER_DOF]= np.reshape(msg.data, (len(msg.data), 1))
    
    def cluster_desired_callback(self, msg):
        self.get_logger().info(f"Received cluster desired position: {msg.data}")
        if self.both:
            if self.actual_configured:
                self.c_des[0:len(msg.data)]= np.reshape(msg.data, (len(msg.data), 1))
            if self.sim_configured:
                self.sim_c_des[0:len(msg.data)]= np.reshape(msg.data, (len(msg.data), 1))
        else:
            if self.output == "actual" and self.actual_configured:
                self.c_des[0:len(msg.data)]= np.reshape(msg.data, (len(msg.data), 1))
            elif self.output == "sim" and self.sim_configured:
                self.sim_c_des[0:len(msg.data)]= np.reshape(msg.data, (len(msg.data), 1))
   
    #Publishes velocity commands to robots in either sim or actual
    def publish_velocities(self, output):
        _msg_prefix = '' if output == 'actual' else '/sim'
        _cluster_robots = self.registered_robots if output == "actual" else self.sim_registered_robots
        #self.get_logger().info(f"Publishing velocities to robots:{_cluster_robots} with output: {output}")
        if not self.enable:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            for i in range(len(_cluster_robots)):
                self.pubsub.publish(f"{_msg_prefix}/{self.robot_id_list[_cluster_robots[i]]}/cmd_vel", vel)
            return
        _cluster = self.cluster
        _r = self.r if output == "actual" else self.sim_r
        _rdot = self.rdot if output == "actual" else self.sim_rdot
        _c_des = self.c_des if output == "actual" else self.sim_c_des
        _cdot_des = self.cdot_des if output == "actual" else self.sim_cdot_des
        cdot_cmd, rdot, c_cur = _cluster.get_velocity_command(_r , _rdot, _c_des, _cdot_des)
        #self.get_logger().info(f"Cluster status/cdot: {cdot.flatten()}")
        #self.get_logger().info(f"Cluster status/rdot: {rdot.flatten()}")
        _rdot = rdot

        #_max = max(np.abs([rdot[0], rdot[1], rdot[3], rdot[4], rdot[6], rdot[7]]))
        #rdot = rdot / _max * MAX_VEL if _max > MAX_VEL else rdot
        rover_vel = []
        for i in range(len(_cluster_robots)):
            _x = float(rdot[i*ROVER_DOF+0, 0])
            _y = float(rdot[i*ROVER_DOF+1, 0])
            _trans = math.sqrt(_x**2 + _y**2)
            _rotate = 0.0
            if _trans < EPSILON*KP_GAIN: #Distance tolerance from robots desired position
                rover_vel.append([0.0, 0.0])
            else:
                desiredAngle = math.atan2(_x, _y)
                self.get_logger().info(f"_x={_x}, _y={_y}, cur_theta={_r[i*ROVER_DOF+2, 0]}, atan2={desiredAngle}")
                _rotate = self.wrap_to_pi(desiredAngle - _r[i*ROVER_DOF+2, 0])
                if abs(_rotate) < math.pi/2:
                    _trans = _trans * math.cos(abs(_rotate))
                else:
                    _rotate = self.wrap_to_pi(math.pi - _rotate)
                    _trans = -_trans * math.cos(abs(_rotate))
                rover_vel.append([_trans, _rotate*3])
        # Limit commanded velocities to max velocity
        rover_vel = np.array(rover_vel)
        for i in range(len(_cluster_robots)):
            rover_vel[i][1] = max(-MAX_VEL, min(MAX_VEL, rover_vel[i][1]))
            limit = MAX_VEL - abs(rover_vel[i][1])
            rover_vel[i][0] = max(-limit, min(limit, rover_vel[i][0]))
        for i in range(len(_cluster_robots)):
            vel = Twist()
            vel.linear.x = rover_vel[i][0]
            vel.angular.z = rover_vel[i][1]
            try:
                self.pubsub.publish(f"{_msg_prefix}/{self.robot_id_list[_cluster_robots[i]]}/cmd_vel", vel)
            except Exception as e:
                self.get_logger().error(f"Failed to publish velocity command: {e}")
            self.get_logger().info(f"Robot {i} velocity[{_msg_prefix}/{self.robot_id_list[_cluster_robots[i]]}/cmd_vel]: {vel.linear.x}, {vel.angular.z}")
        msg = ClusterInfo()
        msg.cluster.data = c_cur.flatten().tolist()
        msg.cluster_desired.data = self.c_des.flatten().tolist()
        msg.rover.data = _r.flatten().tolist()
        msg.rover_desired.data = _rdot.flatten().tolist()
        self.pubsub.publish(f"{_msg_prefix}/cluster_info", msg)
    
    def publish_velocities_manager(self):
        if self.cluster_enable:
            if self.both:
                if self.actual_configured:
                    self.publish_velocities("actual")
                if self.sim_configured:
                    self.publish_velocities("sim")
            if self.output == "actual" and self.actual_configured:
                self.publish_velocities("actual")
            elif self.output == "sim" and self.sim_configured:
                self.publish_velocities("sim")

    def wrap_to_pi(self, t):
        return (t + np.pi) % (2 * np.pi) - np.pi

    #checks if given robot id is in cluster and if not adds it to cluster
    def check_robot_id(self, robot_id, output):
        if output == "actual":
            if robot_id in self.cluster_robots:
                return
            self.cluster_robots.append(robot_id)
            self.pubsub.create_publisher(Twist, f'/{self.robot_id_list[robot_id]}/cmd_vel', 5)
            self.get_logger().info(f"Added robot #{len(self.cluster_robots)} {self.robot_id_list[robot_id]}")
            if not self.actual_configured and len(self.cluster_robots) == self.cluster_size:
                self.assign_robots("actual")
        elif output == "sim":
            if robot_id in self.sim_cluster_robots:
                return
            self.sim_cluster_robots.append(robot_id)
            self.pubsub.create_publisher(Twist, f'/sim/{self.robot_id_list[robot_id]}/cmd_vel', 5)
            self.get_logger().info(f"Added sim robot #{len(self.sim_cluster_robots)} {self.robot_id_list[robot_id]}")
            if not self.sim_configured and len(self.sim_cluster_robots) == self.cluster_size:
                self.assign_robots("sim")
        
    #Maps Id of robot to its index in the cluster
    def map_robot_id(self, i, output):
        if output == "actual":
            for j in range(len(self.cluster_robots)):
                if self.cluster_robots[j] == i:
                    return j
            return None
        elif output == "sim":
            for j in range(len(self.sim_registered_robots)):
                if self.sim_registered_robots[j] == i:
                    return j
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()