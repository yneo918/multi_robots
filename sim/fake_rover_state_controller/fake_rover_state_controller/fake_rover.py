import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D
from pioneer_interfaces.msg import PioneerInfo

from .my_ros_module import PubSubManager

UPDATE_RATE = 0.1
VEL_ALIVE = 10
MAX_TRANS = 1.0
MAX_ROTATE = 1.0


class FakeRover(Node):

    def __init__(self):
        super().__init__('fake_rover')
        self.pubsub = PubSubManager(self)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p0"),
                ('x', 0.0),
                ('y', 0.0),
                ('t', 0.0)
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")

        self.robot_id = self.get_parameter('robot_id').value
        
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        t = self.get_parameter('t').value

        self.position = {'x': x, 'y': y, 'theta': t}
        self.position_hw = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.position_desired = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.vel = {'transform': 0.0, 'rotate': 0.0, 'alive': 0}
        self.joint_names = ['w_to_x', 'x_to_y', 'y_to_t']

        self.pubsub.create_subscription(Twist, f'/{self.robot_id}/cmd_vel', self.teleop_callback, 10)
        self.pubsub.create_publisher(Pose2D, f'/{self.robot_id}/pose2D', 10)

        
        timer_period = UPDATE_RATE
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def update_position(self):
        if self.vel['alive'] <= 0:
            return
        else:
            _theta_avg = self.position['theta'] - self.vel['rotate'] * UPDATE_RATE / 2
            self.position['theta'] -= UPDATE_RATE  * self.vel['rotate']
            self.position['theta'] = self.wrap_to_pi(self.position['theta'])
            self.position['x'] += UPDATE_RATE  * self.vel['transform'] * math.sin(_theta_avg)
            self.position['y'] += UPDATE_RATE  * self.vel['transform'] * math.cos(_theta_avg)
            self.vel['alive'] -= 1
        self.pubsub.publish(f'/{self.robot_id}/pose2D', Pose2D(x=self.position['x'], y=self.position['y'], theta=self.position['theta']))
    
    def wrap_to_pi(self, t):
        return (t + math.pi) % (2 * math.pi) - math.pi
        
    def timer_callback(self):
        self.update_position()

    def teleop_callback(self, msg):
        self.vel.update({'transform': max(-MAX_TRANS, min(msg.linear.x, MAX_TRANS)), 'rotate': max(-MAX_ROTATE, min(msg.angular.z, MAX_ROTATE)), 'alive': VEL_ALIVE})



def main(args=None):
    rclpy.init(args=args)

    teleop_sub = FakeRover()

    rclpy.spin(teleop_sub)

    teleop_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()