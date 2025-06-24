""" THIS IS ONLY USED FOR TESTING ONLY"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D

from adaptive_navigation_utilities.pubsub import PubSubManager

import matplotlib.pyplot as plt
import numpy as np
from typing import Callable, Any

__todo__ = """
- Test with simulation
- Create GUI for changing the Surface plot dynamically
- Add GUI button for changing the Z-desired
- Create dynamic map instead of static map
"""

def ZCallable(x, y) -> np.ndarray:
    return -100 + 100/(1 + np.sqrt(((x/4)**2 + (y/4)**2)))

class Surface3D(Node):

    def __init__(self):
		    
        # Create node
        # Note: We use the __name__ macro and
        # parse for the file name to use as
        # the node name
        super().__init__(__name__.split('.')[-1])

        # Create pubsub manager
        self.pubsub: PubSubManager = PubSubManager(self)

        # Assign surface function
        self.ZCallable: Callable = ZCallable

        # Create static map
        self._create_static_map()

        # Add pubsub topics
        self._add_ros_params()

        # Map ROS params to attributes
        self.robot_id_list = self.get('robot_id_list')

        self._set_pubsub()

        # Add aliases for logging
        self.debug: Callable = self.log().debug
        self.info: Callable = self.log().info
        self.warn: Callable = self.log().warn
        self.error: Callable = self.log().error



    # Crete alias function for common ROS functions
    def get(self, param) -> Any:
        return self.get_parameter(param).value
    
    def log(self):
        return self.get_logger()

    def _add_ros_params(self) -> None:
        """Add ROS Params"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ("z_topic_name", "/sim/rssi"),
                ("robot_id_list", ["p2", "p3", "p4"]),
            ]
        )

    def _set_pubsub(self):

        for robot_id in self.robot_id_list:
            self.pubsub.create_subscription(
                Pose2D,
                f'/{robot_id}/pose2D',
                lambda msg, robot_id=robot_id: self.return_z(msg, robot_id),
                5)
            self.pubsub.create_subscription(
                Pose2D,
                f'/sim/{robot_id}/pose2D',
                lambda msg, robot_id=robot_id: self.return_sim_z(msg, robot_id),
                5)
            self.pubsub.create_publisher(
                Int16,
                f'/{robot_id}/rssi',
                5)
            self.pubsub.create_publisher(
                    Int16,
                    f'/sim/{robot_id}/rssi',
                    5)

    def return_sim_z(self, msg, robot_id):
        z = int(self.ZCallable(msg.x, msg.y))
        res = Int16(data=z)
        self.pubsub.publish(f'/sim/{robot_id}/rssi', res)


    def return_z(self, msg, robot_id):
        z = int(self.ZCallable(msg.x, msg.y))
        res = Int16(data=z)
        self.pubsub.publish(f'/{robot_id}/rssi', res)

    def _create_static_map(self) -> None:
        x = np.linspace(-3, 3, 100)
        y = np.linspace(-3, 3, 100)
        X, Y = np.meshgrid(x, y)

        Z = self.ZCallable(X, Y)

        fig = plt.figure()

        ax = fig.add_subplot(111, projection='3d')
        ax.contour3D(X, Y, Z, 50, cmap='coolwarm')
        ax.set_title("3D Contour Plot")
        plt.show()

def main(args=None):

		# Initialize ros client library
    rclpy.init(args=args)

		# Create publisher node
    surf3d = Surface3D()
		
		# Continuously blocking poll
    rclpy.spin(surf3d)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    surf3d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()