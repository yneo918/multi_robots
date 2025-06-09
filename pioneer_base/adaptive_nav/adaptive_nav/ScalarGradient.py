import numpy as np
import math
import sympy as sp 
from enum import Enum

class ControlMode(Enum):
    """Different control modes for the cluster"""
    MAX = "maximum"
    MIN = "minimum"
    CONTOUR = "contour following"
    
    def __str__(self):
        return self.value

class ScalarGradient:
    """
    Adaptive Navigation Scalar Gradient Class
    
    This class implements the scalar gradient control for a cluster of robots.
    It allows for different control modes such as maximum, minimum, and contour following.
    """
    def __init__(self, num_robots=3, mode=ControlMode.MAX):
        self.num_robots = num_robots

        # Array of robot position vectors in [x, y, z] where x, y is position in the plane
        # and z is the "height" in the scalar field.
        self.robot_positions = [self.num_robots*[0.0] for _ in range(3)] 

        self.control_mode = mode
        self.direction = 0 if mode == ControlMode.MAX else 1  # 0 for max, 1 for min
        self.rotation = 1 # 1 for clockwise, -1 for counter-clockwise
        self.zdes = 0.0 #Desired height for contour following

    def get_velocity(self) -> [float]:
        bearing = self.calc_gradient()
        # Determine velocity based on the bearing and mode [ascent, descent, contour folowing]
        return None

    def calc_gradient(self) -> float:
        #math here to return gradient bearing
        return 0.0  # Placeholder for actual gradient calculation