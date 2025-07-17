# movebase_kinematics.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from math import pi
import os


class GetMoveCmds(Node):

    def __init__(self):
        # Get robot ID from parameter or environment
        robot_id = os.getenv("ROBOT_ID", "pX")
        super().__init__(f'{robot_id}_movebase_kinematics')
        
        # Declare parameters with defaults from system_config.yaml structure
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', robot_id),
                ('max_vel', 50),
                ('max_vel_open', 600),
                ('left_motor_sign', -1),  # Motor direction signs
                ('right_motor_sign', 1)
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.max_vel = self.get_parameter('max_vel').get_parameter_value().integer_value
        self.max_vel_open = self.get_parameter('max_vel_open').get_parameter_value().integer_value
        self.left_motor_sign = self.get_parameter('left_motor_sign').get_parameter_value().integer_value
        self.right_motor_sign = self.get_parameter('right_motor_sign').get_parameter_value().integer_value
        
        # Validate motor signs
        if self.left_motor_sign not in [-1, 1]:
            self.get_logger().warn(f'Invalid left_motor_sign: {self.left_motor_sign}, using -1')
            self.left_motor_sign = -1
        if self.right_motor_sign not in [-1, 1]:
            self.get_logger().warn(f'Invalid right_motor_sign: {self.right_motor_sign}, using 1')
            self.right_motor_sign = 1
        
        # Log loaded configuration
        self.get_logger().info(f'Movebase Kinematics Configuration:')
        self.get_logger().debug(f'  Robot ID: {self.robot_id}')
        self.get_logger().debug(f'  Max Velocity: {self.max_vel}')
        self.get_logger().debug(f'  Max Velocity (Open Loop): {self.max_vel_open}')
        self.get_logger().debug(f'  Left Motor Sign: {self.left_motor_sign}')
        self.get_logger().debug(f'  Right Motor Sign: {self.right_motor_sign}')

        # Create a subscription to the 'cmd_vel' topic with a callback function 
        self.subscription = self.create_subscription(
            Twist,
            f'/{self.robot_id}/cmd_vel',
            self.move_cmd_callback, 
            5)
        self.subscription  

        # Create a publisher for the left & right wheel's control signal on their topics
        self.pub_move = self.create_publisher(
            Int32MultiArray, 
            f'/{self.robot_id}/ch_vals',
            5)

        self.lx = 0.0
        self.az = 0.0
        self.vel_left = 0
        self.vel_right = 0

    def move_cmd_callback(self, msg):
        # Update 'linear_x' and 'angular_z' with the linear and angular commands from the message
        lx = msg.linear.x
        az = msg.angular.z

        # Calculate control signals based on 'linear_x' and 'angular_z' values 
        # Apply motor direction signs from configuration
        vel_left  = self.left_motor_sign * int(self.max_vel * (lx - az))
        vel_right = self.right_motor_sign * int(self.max_vel * (lx + az))

        # Construct payload with left & right velocities 
        payload = Int32MultiArray()
        payload.data = [vel_left, vel_right]
    
        # Publish the message on the 'ch_vals' topic
        self.pub_move.publish(payload)


def main(args=None):
    # Initialize the ROS2 node
    rclpy.init(args=args)
    # Create an instance of the 'GetMoveCmds' class, which starts the subscription and timers
    sub_move_cmds = GetMoveCmds()
    # Enter the ROS2 event loop and wait for callbacks to be triggered
    rclpy.spin(sub_move_cmds)
    # Clean up and destroy the node when the event loop exits
    sub_move_cmds.destroy_node()
    # Shutdown the ROS2 client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()