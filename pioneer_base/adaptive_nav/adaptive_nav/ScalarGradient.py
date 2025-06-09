import numpy as np
import math
from enum import Enum
from typing import List, Literal, Tuple, Optional


class ControlMode(Enum):
    """Different control modes for the cluster"""
    MAX = ("maximum", 0.0, 0.0)
    MIN = ("minimum", math.pi, 0.0)
    CONTOUR_CW = ("contour following clockwise", -math.pi/2, 0.0)
    CONTOUR_CCW = ("contour following counter-clockwise", math.pi/2, 0.0)
    CROSSTRACK_CW = ("crosstrack_cw_controller", -math.pi/2, 1.0) 
    CROSSTRACK_CCW = ("crosstrack_ccw_controller", math.pi/2, 1.0)

    def __init__(self, value: str, bearing_offset: float, gain: Optional[float]):
        self._value_ = value
        self.bearing_offset = bearing_offset
        self.gain = gain
    
    def __str__(self):
        return self.value
    

class ClusterSize(Enum):
    THREE = 3
    FIVE = 5

    def __str__(self):
        return self.value


class ScalarGradient:
    """
    Adaptive Navigation Scalar Gradient Class
    
    This class implements the scalar gradient control for a cluster of robots.
    It allows for different control modes such as maximum, minimum, and contour following.
    """
    def __init__(
                self, 
                 num_robots: ClusterSize = ClusterSize.THREE.value, 
                 mode: ControlMode = ControlMode.MAX
                 ):
        
        self.num_robots: ClusterSize = num_robots.value

        # Array of robot position vectors in [x, y, z] where x, y is position in the plane
        # and z is the "height" in the scalar field.
        self.robot_positions: List[List[float]] = \
                                np.array([self.num_robots*[0.0] for _ in range(3)])

        # Control mode for Adaptive Navigation
        self.mode: ControlMode = mode

        # Gradient scalar parameters
        self.zdes: float = 0.0 #Desired height for contour following
        self.z_err: float = 0.0 # Cross track error
        self.grad: List[float] = list()
        self.curr_bearing = 0.0
        self.des_bearing = 0.0

        # List of 3 of the robots to form planar vectors
        # The first index is the tail, while the remaining second and third
        # indices are the heads of the vectors.
        #
        # By default the 3 robots will be the first robot, the middle robot,
        # and the last robot
        #
        # Example: For a 3-robot cluster, the robot_reference_list will be
        #                                                       (0, 1, 2)
        # 
        # Example: For a 5-robot cluster, the robot_reference_list will be
        #                                                       (0, 2, 4)
        self.robot_reference_list: Tuple[int, int, int] = \
            (0, self.num_robots//2, self.num_robots - 1)
        
    @property
    def direction(self) -> Optional[Literal[0,1]]:

        if self.mode not in (
            ControlMode.MAX,
            ControlMode.MIN
        ):
            return None

        return 0 if self.mode == ControlMode.MAX else 1
        
        
    @property
    def rotation(self) -> Optional[Literal[-1, 1]]:

        if self.mode not in (
        ControlMode.CONTOUR_CW,
        ControlMode.CONTOUR_CCW,
        ControlMode.CROSSTRACK_CW,
        ControlMode.CROSSTRACK_CCW,
        ):
            return 0

        return 1 if self.mode in (
            ControlMode.CONTOUR_CW,
            ControlMode.CROSSTRACK_CW,
        ) else -1

    @property
    def tail(self) -> int:
        return self.robot_reference_list[0]
    
    @property 
    def heads(self) -> List[int]:
        return self.robot_reference_list[1:]
    
    @property
    def z_c(self) -> float:

        # Get average of the z value across all robots
        return np.average([self.robot_positions[i][-1] \
                           for i in range(self.num_robots)])
    
    
    @staticmethod
    def sign(expr) -> Literal[-1, 0, 1]:
        try:
            if expr > 1:
                return 1
            if expr < 1:
                return -1
            else:
                return 0
        except Exception as e:
            print(f"Invalid expression: {e}")

    
    def get_velocity(self, robot_positions: List[List[float]] = None, zdes: float = None) -> Optional[float]:

        # Assign to self if external params given
        if zdes != None: 
            self.zdes = zdes

        try:
            # Calculate gradient from robot positions
            grad: Tuple[float, float] = self.calc_gradient(robot_positions)

            # Calculate the bearing angle
            # NOTE (1): PI/2 - ATAN2() returns the complement angle
            # NOTE (2): grad[0], grad[1], is the x- and y- component
            current_bearing: float = math.pi/2 - math.atan2(grad[1], grad[0])

            # Update internal param
            self.curr_bearing = current_bearing

                
            # Based on the current mode, change the angle by a fixed offset
            # For Example: the offset for maximum/increasing gradient is 0,
            #              but for minimum/decreasing gradient, PI is added to 
            #              flip the direction.
            # Get crosstrack error
            self.z_err: float = self.zdes - self.z_c

            # Generalized Controller for all modes, including and excluding
            # cross-track control
            desired_bearing: float = current_bearing + self.mode.bearing_offset \
            + self.rotation * (
                self.sign(self.z_err) * min(self.mode.gain * abs(self.z_err), math.pi/2)
            )

            # Update internal param
            self.des_bearing = desired_bearing

            return desired_bearing
            
        except Exception as e:
            print(f"Could not calculate bearing: {e}")




    def calc_gradient(self, robot_positions: List[List[float]] = None) -> List[float]:
        
        # Assign to self if external params given
        if robot_positions != None:
            self.robot_positions = np.array(robot_positions)

        try:

            # TODO: Expand this logic for 5-cluster configuration
            # TODO: How do we determine which robots to use? 1 and 2,
            #                                    2 and 3, or 1 and 3?
            #       Can we take an average of this?
            # Step 1: Extract the planar vectors R_12 and R_23
            # Initialize planar vectors
            vectors: List[np.ndarray] = [[0.0]*self.num_robots for _ in range(2)]

            # First vector
            
            print(self.robot_positions[0])

            vectors[0] = self.robot_positions[self.heads[0]] \
                            - self.robot_positions[self.tail] 

            # Second vector
            vectors[1] = self.robot_positions[self.heads[1]] \
                            - self.robot_positions[self.tail] 
            
            
            # Step 2: Use cross product of planar vectors to form gradient
            # NOTE (1): .tolist() is used as it is up to the user of this module
            #           to typecast the gradient to any desired Iterable object
            # NOTE (2): [:2] is used to extract the x- and y-components of the 
            #           the gradient for navigation. The third field is not
            #           needed for navigation.
            grad: np.ndarray = (- np.cross(vectors[0], vectors[1])).tolist()[:2]

            # Udpate internal param
            self.grad = grad
        
        except Exception as e:

            print(f"Could not calculate gradient: {e}")

        return grad 
    
if __name__ == "__main__":

    # Example of 3-cluster configuration performing cross-track
    grad: ScalarGradient = ScalarGradient(
                    ClusterSize.THREE, 
                    ControlMode.MIN)
    

    example_robot_pos_list: List[List[float]] = \
    [[1.0, 2.0, -8.0],
     [2.2, -3.0, -10.0],
     [0.0, 1.6, -9.0]]
    

    
    vel = grad.get_velocity(example_robot_pos_list, -10.0)
    print(f"control mode: {grad.mode}")
    print(f"grad: {grad.grad}")
    print(f"current bearing: {grad.curr_bearing}")
    print(f"z_c: {grad.z_c}")
    print(f"z_err: {grad.z_err}")
    print(f"crosstrack gain: {abs(grad.z_err)*grad.mode.gain}")
    print(f"desired bearing: {vel}")