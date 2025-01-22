import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16


class Multiplexer(Node):

    def __init__(self):
        super().__init__('rover2_teleop')
        self.subscription = self.create_subscription(
            Twist,
            '/joy/cmd_vel',
            self.cmd_callback,
            5
            )
        self.subscription = self.create_subscription(
            Int16,
            '/toggle',
            self.toggle_callback,
            4
            )
        self.subscription  # prevent unused variable warning
        self.N_ROVER = 6  # number of rovers
        self.selected_roverN = 0
        
        self.publisher_cmd_vel = []
        for i in range(self.N_ROVER):
            self.publisher_cmd_vel.append(self.create_publisher(Twist, f'/p{i+1}/cmd_vel', 5))
   
    def toggle_callback(self, msg):
        self.selected_roverN = msg.data

    def cmd_callback(self, msg):
        val = []
        for i in range(1, self.N_ROVER+1):
            if i == self.selected_roverN:
                val.append(msg)
            else:
                v = Twist()
                v.linear.x = 0.0
                v.angular.z = 0.0
                val.append(v)
        
        for i in range(self.N_ROVER):
            self.publisher_cmd_vel[i].publish(val[i])
    
    def timer_callback(self):
        selected_roverN = Int16()
        selected_roverN.data = self.selected_roverN+1
        self.publisher_toggle.publish(selected_roverN)


def main(args=None):
    rclpy.init(args=args)
    toggle_handle = Multiplexer()
    rclpy.spin(toggle_handle)
    toggle_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
