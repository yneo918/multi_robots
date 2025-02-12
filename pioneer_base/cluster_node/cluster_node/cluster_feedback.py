import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
import random
from teleop_core.my_ros_module import PubSubManager

class ClusterFeedbackNode(Node):
    def __init__(self):
        super().__init__('cluster_feedback')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"])
            ]
        )
        self.robot_id_list = self.get_parameter('robot_id_list').value
        self.pubsub = PubSubManager(self)
        for robot_id in self.robot_id_list:
            self.pubsub.create_publisher(Float32MultiArray, f'/{robot_id}/imu/eulerAngle', 10)
            self.pubsub.create_publisher(NavSatFix, f'/{robot_id}/gps1', 10)
        
        self.timer = self.create_timer(0.1, self.publish_feedback)  # 10 Hz

    def publish_feedback(self):
        #self.get_logger().info(f"Publishing from: {self.robot_id_list}")
        for robot_id in self.robot_id_list:
            #self.get_logger().info(f"Publishing from: {robot_id}")
            # Create and publish IMU data
            imu_msg = Float32MultiArray()
            imu_msg.data = [random.uniform(-180, 180), random.uniform(-180, 180), random.uniform(-180, 180)]
             #self.get_logger().info(f"Publishing from: {robot_id}")
            self.pubsub.publish( f'/{robot_id}/imu/eulerAngle', imu_msg)

            # Create and publish GPS data
            gps_msg = NavSatFix()
            gps_msg.latitude = random.uniform(-90, 90)
            gps_msg.longitude = random.uniform(-180, 180)
            gps_msg.altitude = random.uniform(0, 10000)
            self.pubsub.publish(f'/{robot_id}/gps1', gps_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClusterFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()