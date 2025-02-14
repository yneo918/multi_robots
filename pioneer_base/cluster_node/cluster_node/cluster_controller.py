import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from .Cluster import Cluster
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

from teleop_core.my_ros_module import PubSubManager
#manage active robot IDs 
#map robot IDs to robot state space variables
class ClusterNode(Node):
    def __init__(self, n_rover=6):
        super().__init__('cluster_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"])
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")
        
        #Parse robot ID list
        robot_id_list = self.get_parameter('robot_id_list').value
        self.get_logger().info(f"robot_id_list: {robot_id_list}")
        if len(robot_id_list) == 1 and robot_id_list[0] == "p0":
            robot_id_list = None
        self.robot_id_list = []
        if robot_id_list is None:
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"{self.robot_id_list[i]}")
        elif isinstance(robot_id_list, int):
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"p{i+1+robot_id_list}")
        elif isinstance(robot_id_list, list):
            self.n_rover = len(robot_id_list)
            if isinstance(robot_id_list[0], int):
                for i in range(self.n_rover):
                    self.robot_id_list.append(f"p{robot_id_list[i]}")
            else:
                self.robot_id_list = robot_id_list
        else:
            print("ROBOT_ID_LIST ERROR")
            return
        self.get_logger().info(f"ROVER: {self.robot_id_list} N: {self.n_rover}")

        self.cluster = Cluster(KPgains=[0.25]*9, KVgains=[0.25]*9)
        self.robots = 0
        self.cluster_robots = []
        self.world_frame = NavSatFix()
        self.gpsStartup = []
        self.r = np.zeros((9, 1)) #stores latest robot state space variables
        self.rd = np.zeros((9, 1)) #stores latest robot velocities commands
        
        
        self.pubsub = PubSubManager(self)
        
        for i in range(self.n_rover):
            self.pubsub.create_subscription(
                Float32MultiArray,
                f'/{self.robot_id_list[i]}/imu/eulerAngle',
                lambda msg, i=i: self.angular_callback(msg, i),
                5)
            self.pubsub.create_subscription(
                NavSatFix,
                f'/{self.robot_id_list[i]}/gps1',
                lambda msg, i=i: self.gps_callback(msg, i),
                5)
        self.listeningForRobots = True
        self.timer_once = self.create_timer(2.0, self.assignRobots)
        
    def assignRobots(self):
        self.listeningForRobots = False #done listening for robots
        self.cluster_robots.sort()
        self.robots = len(self.cluster_robots)
        self.r = np.zeros((self.robots*3, 1))
        self.rd = np.zeros((self.robots*3, 1))
        self.get_logger().info(f"Cluster status: {self.cluster.cdes}")
        self.get_logger().info(f"Formed cluster with robots: {self.cluster_robots} from list of possible: {self.robot_id_list}")
        #average all read gps values to assign rough world frame
        for coord in self.gpsStartup:
            self.world_frame.latitude += coord.latitude
            self.world_frame.longitude += coord.longitude
        if(len(self.gpsStartup) > 0):
            self.world_frame.latitude /= len(self.gpsStartup)
            self.world_frame.longitude /= len(self.gpsStartup)
        self.world_frame.latitude = 37.774932  # Example reference latitude
        self.world_frame.longitude = -122.419467 
        self.get_logger().info(f"World frame coords: { self.world_frame.latitude}, { self.world_frame.longitude}")
        self.timer_once.cancel()
        timer_period = 1  # set frequency to publish velocity commands
        for i in range(self.n_rover):
            self.pubsub.create_publisher(Twist, f'{self.robot_id_list[i]}/cmd_vel', 5)
        self.timer = self.create_timer(timer_period, self.publish_velocities)

    #checks if given robot id is in cluster and if not adds it to cluster
    def checkRobotId(self, id):
        #self.get_logger().info(f"Recieved message from: {id}")
        for robot in self.cluster_robots:
            if robot == id:
                return
        self.cluster_robots.append(id)

    def angular_callback(self, msg, i):
        #self.get_logger().info(f"{self.robot_id_list[i]}/ Received angular vel: {msg.data[-1]}") 
        if self.listeningForRobots:
            self.checkRobotId(i)
        else:
            cluster_index = self.mapRobotId(i)
            if cluster_index is None:
                return
            self.r[cluster_index*3+2, 0] = msg.data[-1] #update robot heading in array

    def gps_callback(self, msg, i):
        #self.get_logger().info(f"{self.robot_id_list[i]}/ Received gps coords: {msg.latitude}, {msg.longitude} ") 
        if self.listeningForRobots:
            self.checkRobotId(i)
            self.gpsStartup.append(msg)
        else:
            cluster_index = self.mapRobotId(i)
            if cluster_index is None:
                return
            x, y = self.gps_to_xy(self.world_frame.latitude, self.world_frame.longitude, msg.latitude, msg.longitude)
            self.r[cluster_index*3:(cluster_index*3+2), 0] = [x, y] #update robot position in array
            self.get_logger().info(f"{self.robot_id_list[i]}/ Updated robot position: {self.r[cluster_index*3:(cluster_index*3+2), 0]}")

    #Maps Id of robot to its index in the cluster
    def mapRobotId(self, i):
        for j in range(len(self.cluster_robots)):
            if self.cluster_robots[j] == i:
                return j
        return None

    #Publishes velocity commands to robots
    def publish_velocities(self):
        rd = self.cluster.getVelocityCommand(self.r , self.rd)
        for i in range(len(self.cluster_robots)):
            vel = Twist()
            vel.linear.x = float(rd[i*3+0, 0])
            vel.linear.y = float(rd[i*3+1, 0])
            self.pubsub.publish(f'{self.robot_id_list[self.cluster_robots[i]]}/cmd_vel', vel)
        #self.get_logger().info(f"Cluster updated position: {self.cluster.c}")
        #self.get_logger().info(f"Cluster updated velocity: {self.cluster.cd}")
        #self.get_logger().info(f"Cluster control output: {self.cluster.calculateLinearControl()}")

            #self.get_logger().info(f"{self.robot_id_list[i]}/ Sending velocity command: {vel} ") 
            #self.pubsub.publish(f'{self.robot_id_list[i]}/enable', en_state)

    #Lat1 Lon1 are world frame coordinates
    def gps_to_xy(self, lat1, lon1, lat2, lon2):
        R = 6371000   # Earth radius in meters
        # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # Calculate bearing
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.atan2(y, x)
        x = distance * math.sin(bearing)
        y = distance * math.cos(bearing)

        return x, y

def main(args=None):
    rclpy.init(args=args)
    node = ClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
