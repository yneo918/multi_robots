import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16


class Multi_process(Node):

    def __init__(self):
        super().__init__('rover2_teleop')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.lx_axisN = 1
        self.az_axisN = 3
        self.en_buttonN = 4 # LB
        self.TOGGLE_BUTTON = 3 # Y
        self.N_ROVER = 6  # number of rovers
        self.selected_roverN = 1
        self.toggle_prev = 0
        
        self.publisher_cmd_vel = self.create_publisher(Twist, '/joy/cmd_vel', 5)
        self.publisher_en = self.create_publisher(Bool, '/enable', 1)
        self.publisher_toggle = self.create_publisher(Int16, '/toggle', 4)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #self.j_lx_lx = 0
        #self.j_az = 0 
    
    
    def joy_callback(self, msg):

        j_lx = msg.axes[self.lx_axisN]
        j_az = msg.axes[self.az_axisN]
        val = Twist()

        en_state = Bool()
        en_state.data = False
        if msg.buttons[self.en_buttonN] == 1:
            en_state.data = True
            val.linear.x = 0.7*j_lx
            val.angular.z = 0.5*j_az
        else:
            en_state.data = False
            val.linear.x = 0.0
            val.angular.z = 0.0
        
        if msg.buttons[self.TOGGLE_BUTTON] == 1 and self.toggle_prev == 0:
            self.selected_roverN = (self.selected_roverN)%self.N_ROVER + 1

        self.toggle_prev = msg.buttons[self.TOGGLE_BUTTON]

        self.publisher_cmd_vel.publish(val)
        self.publisher_en.publish(en_state)
    
    def timer_callback(self):
        selected_roverN = Int16()
        selected_roverN.data = self.selected_roverN
        self.publisher_toggle.publish(selected_roverN)


def main(args=None):
    rclpy.init(args=args)
    multi_handle = Multi_process()
    rclpy.spin(multi_handle)
    multi_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
