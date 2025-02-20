import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String

import time

global joy_lx, joy_az, u_lx, u_az, ply_lx, ply_az, deadmanButtonState 

joy_lx = 0.0; joy_az = 0.0; ply_lx = 0.0; ply_az = 0.0; u_lx = 0.0; u_az = 0.0

deadmanButtonState = False

class get_move_cmds(Node):

    def __init__(self):
        super().__init__('rover_state_controler_r2')
        self.subscription = self.create_subscription(
            Twist,
            'r2/joy/cmd_vel',
            self.joy_cmd_callback,
            5)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            5)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Twist,
            '/r2/cmd_vel',
            self.ply_cmd_callback,
            5)
        self.subscription  # prevent unused variable warning

        self.toggle_button = 0  # Toggle button to cycle between states.
        self.rover_modeC = "NEU_M"  # Assigned state of the rover.
        self.toggle_flag = 0     # if flag = 1; locked, flag = 0; free

        self.pub_core_cmdvel = self.create_publisher(Twist, 'r2/core_cmdvel', 5)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.core_cmdvel_callback)
        self.i = 0

        # self.pub_rover_en = self.create_publisher(Bool, 'r4/enable', 1)
        # timer_period = 0.2  # seconds
        # self.timer = self.create_timer(timer_period, self.rover_en_callback)
        # self.i = 0

        self.pub_robot_modeC = self.create_publisher(String, 'r2/modeC', 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.robot_modeC_callback)
        self.i = 0
        self.prev_state = False

        ## Client initialization section.
        #self.en_cli = self.create_client(SetBool, '/rov/en')
        #while not self.en_cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Service not avail, waiting again ...')
        #self.req  = SetBool.Request()
        
    def toggle(self, state):
        if state == 0:
            self.rover_modeC = "NEU_M"
            self.toggle_button = 1
        elif state == 1:
            self.rover_modeC = "TEA_M"
            self.toggle_button = 2
        else:
            self.rover_modeC = "PLY_M"
            self.toggle_button = 0
        
    def joy_callback(self, data):

        #global deadmanButtonState
        #deadmanButtonState = False
        #if (data.buttons[4] == 1 or data.buttons[5] == 1):
        #    if (self.rover_modeC == "TEA_M" or self.rover_modeC == "PLY_M"):
        #        deadmanButtonState = True
        #        self.prev_state = deadmanButtonState
        #        self.req.data = deadmanButtonState
        #        self.en_cli.call_async(self.req)
        #if self.prev_state == True:
        #    self.prev_state = deadmanButtonState
        #    self.req.data = deadmanButtonState
        #    self.en_cli.call_async(self.req)
        #print(self.req.data)
        toggle_button = data.buttons[0]
        if toggle_button == 1:
            if self.toggle_flag == 0:
                self.toggle(self.toggle_button)
        #        print(self.rover_modeC)
                self.toggle_flag = 1
        else:
            self.toggle_flag = 0
        
    def joy_cmd_callback(self, msg):
        global joy_lx, joy_az
        joy_lx = msg.linear.x
        joy_az = msg.angular.z
    
    def ply_cmd_callback(self, msg):
        global ply_lx, ply_az
        ply_lx = msg.linear.x
        ply_az = msg.angular.z
    
    def core_cmdvel_callback(self):
        global u_lx, u_az, joy_lx, joy_az, ply_lx, ply_az
        msg = Twist()

        if (self.rover_modeC == "TEA_M"):
            u_lx = joy_lx
            u_az = joy_az
        
        elif (self.rover_modeC == "PLY_M"):
            u_lx = ply_lx
            u_az = ply_az
        
        elif (self.rover_modeC == "NEU_M"):
            u_lx = 0.0
            u_az = 0.0
        
        msg.linear.x = u_lx
        msg.angular.z = u_az
        self.pub_core_cmdvel.publish(msg)
        self.i += 1

    def robot_modeC_callback(self):
        msg = String()
        msg.data = self.rover_modeC
        self.pub_robot_modeC.publish(msg)
        self.i += 1

    #def rover_en_callback(self):
    #    global deadmanButtonState
    #    msg = Bool()
    #    msg.data = deadmanButtonState
    #    self.pub_rover_en.publish(msg)
    #    self.i += 1

def main(args=None):
    rclpy.init(args=args)

    sub_move_cmds = get_move_cmds()
    rclpy.spin(sub_move_cmds)

    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
