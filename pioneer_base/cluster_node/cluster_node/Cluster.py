import numpy as np
import math
import sympy as sp 
from enum import Enum

class ClusterConfig(Enum):
    TRICEN = "TriangleatCentroid"
    TRILEAD = "TriangleatLeader"
    ELLIPSE = "Ellipse"
    DIAMOND = "Diamond"
    LINE = "Line"
    
    def __str__(self):
        return self.value
"""
The Cluster Class node contains all the cluster state space variables and kinematic transforms. The cluster configuration is determined by the
parameters at startup but may be able to be dynamically adjusted in future once we have defined more configurations.
@params:
    numRobots: Number of robots in the cluster
    clusterType: The geometric shape representing configuration of the cluster (TRICEN is the only one implemented currently)
    clusterParams: array of cluster configuration parameters dependent on configuration (for TRICEN it is [p, q, B] which is the SSA of the triangle)
    KPgains: Proportional gains for the cluster control (must be length numRobots*DOF where DOF is 3 for land rovers)
    KVgains: Derivative gains for the cluster control (must be length numRobots*DOF where DOF is 3 for land rovers)
    control_mode: The control mode for the cluster (POS or VEL)
"""

ROVER_DOF = 3

class Cluster():
    #Create a cluster with a set number of robots in a specific configuration with initial parameters
    def __init__(self, numRobots=3, clusterType=ClusterConfig.TRICEN, clusterParams=[10, 10, math.pi/3], KPgains=None, KVgains=None, control_mode="POS"):
        self.num_robots = numRobots
        self.cluster_type = clusterType
        self.cluster_params = clusterParams
        self.control_mode = control_mode
        try:
            if KPgains is None:
                KPgains = [1.0] * (self.num_robots*ROVER_DOF)
            if KVgains is None:
                KVgains = [1.0] * (self.num_robots*ROVER_DOF)
            assert KPgains is not None and len(KPgains) == self.num_robots*ROVER_DOF, "KPgains must be a list of length self.num_robots*3"
            assert KVgains is not None and len(KVgains) == self.num_robots*ROVER_DOF, "KVgains must be a list of length self.num_robots*3"
        except AssertionError as e:
            print(f"AssertionError: {e}")
            return
        self.Kp = np.diag(KPgains) #nm*nm diagonal matrix of gains
        self.Kv = np.diag(KVgains) #nm*nm diagonal matrix of gains
        self.cluster_angle_index = []
        self.initialize_cluster()
    
    def initialize_cluster(self):
        #Desired cluster state space variables
        self.c_des = np.zeros((self.num_robots*ROVER_DOF, 1))
        self.c_des[(self.num_robots-1)*ROVER_DOF:(self.num_robots)*ROVER_DOF] = np.reshape(self.cluster_params, (self.num_robots, 1))
        self.cd_des = np.zeros((self.num_robots*ROVER_DOF, 1))
        #Actual Cluster state space variables
        self.c = np.zeros((self.num_robots*ROVER_DOF, 1))  
        self.cd = np.zeros((self.num_robots*ROVER_DOF, 1))  
        #Symbolic kinematic transform vars
        self.FKine_func, self.IKine_func, self.Jacobian_func, self.JacobianInv_func = None, None, None, None
        self.configureCluster(self.num_robots, self.cluster_type)

    #Given the current robot state space variables and desired cluster state space variables, calculate robot velocity vector
    def get_velocity_command(self, r, rd):
        self.update_cluster_position(r, rd)
        if self.control_mode == "POS":
            cd_cmd = self.calculate_linear_control()
        elif self.control_mode == "VEL":
            cd_cmd = self.cd_des
        _rd = np.dot(np.array(self.JacobianInv_func(*self.c.flatten())), cd_cmd)
        for i in range(self.num_robots):
            _rd[i*ROVER_DOF+2, 0] = self.wrap_to_pi(rd[i*ROVER_DOF+2, 0])
        return cd_cmd, _rd, self.c
    
    def update_cdes_tr(self, v_t, v_r, freq):
        t = self.c[2, 0]
        self.c_des[0, 0] += v_t*math.sin(t) / freq
        self.c_des[1, 0] += v_t*math.cos(t) / freq
        self.c_des[2, 0] += v_r / freq
        self.c_des[2, 0] = self.wrap_to_pi(self.c_des[2, 0])
        return
    
    def update_cdes_vel(self, v_x, v_y, v_r, freq):
        self.c_des[0, 0] += v_x / freq
        self.c_des[1, 0] += v_y / freq
        self.c_des[2, 0] += v_r / freq
        self.c_des[2, 0] = self.wrap_to_pi(self.c_des[2, 0])
        return
    
    # data = float list
    def update_cdes_pos(self, data):
        for i in range(len(data)):
            self.c_des[i, 0] = data[i]
            if i in self.cluster_angle_index:
                self.c_des[i, 0] = self.wrap_to_pi(self.c_des[i, 0])
        return
    
    def update_cluster_shape(self, params):
        if len(params) != self.num_robots:
            raise ValueError("params must be a list of length self.num_robots*ROVER_DOF")
        self.c_des[(self.num_robots-1)*ROVER_DOF:(self.num_robots)*ROVER_DOF] = np.reshape(params, (self.num_robots, 1))

    def wrap_to_pi(self, t):
        return (t + np.pi) % (2 * np.pi) - np.pi

    #Linear control equation 
    def calculate_linear_control(self):
        c_diff = self.c_des - self.c
        for i in self.cluster_angle_index:
            c_diff[i, 0] = self.wrap_to_pi(c_diff[i, 0])
        cd = np.dot(self.Kp, c_diff)
        return cd

    #Given a the current robot state space variables, update the cluster state space variables
    def update_cluster_position(self, r, rd):
        self.c = self.FKine_func(*r.flatten())
        for i in self.cluster_angle_index:
            self.c[i, 0] = self.wrap_to_pi(self.c[i, 0])
        self.cd = np.dot(np.array(self.Jacobian_func(*r.flatten())).astype(np.float64), rd)

    #Based on the given cluster configuration will set the symbolic kinematic transform equations
    #from a set of prederived transforms
    def configureCluster(self, robots, clusterType):
        #Triangle configuration with cluster at centroid
        if(robots==3):
            if (clusterType == ClusterConfig.TRICEN):
                self.cluster_config_tricen()

    def cluster_config_tricen(self):
        r_sym = sp.symbols('r0:9') #symbols for robot space state variables
        c_sym = sp.symbols('c0:9') #symbols for cluster space state variables

        # cluster space configuration by Robot space variables
        x_c = (r_sym[0] + r_sym[3] + r_sym[6]) / 3
        y_c = (r_sym[1] + r_sym[4] + r_sym[7]) / 3
        theta_c = sp.atan2(2 / 3 * r_sym[0] - 1 / 3 * (r_sym[3] + r_sym[6]), 2 / 3 * r_sym[1] - 1 / 3 * (r_sym[4] + r_sym[7]))
        phi1 = r_sym[2] + theta_c
        phi2 = r_sym[5] + theta_c
        phi3 = r_sym[8] + theta_c
        p = sp.sqrt((r_sym[0]-r_sym[3])**2 + (r_sym[1]-r_sym[4])**2)
        q = sp.sqrt((r_sym[6]-r_sym[0])**2 + (r_sym[1]-r_sym[7])**2)
        B = sp.acos((p**2 + q**2 - (r_sym[6]-r_sym[3])**2 - (r_sym[7]-r_sym[4])**2)/(2*p*q))

        # robot space configuration by cluster space variables
        r = sp.sqrt((c_sym[7]+c_sym[6]*sp.cos(c_sym[8]))**2 + (c_sym[6]*sp.sin(c_sym[8]))**2)
        x_1 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2])
        y_1 = c_sym[1] + 1/3 * r * sp.cos(c_sym[2])
        theta_1 = c_sym[3] - c_sym[2]
        x_2 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2]) - c_sym[6] * sp.sin(c_sym[8]/2 + c_sym[2])
        y_2 = c_sym[1] + 1/3 * r * sp.cos(c_sym[2]) - c_sym[6] * sp.cos(c_sym[8]/2 + c_sym[2])
        theta_2 = c_sym[4] - c_sym[2]
        x_3 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2]) + c_sym[7] * sp.sin(c_sym[8]/2 - c_sym[2])
        y_3 = c_sym[1] + 1/3 * r * sp.cos(c_sym[2]) - c_sym[7] * sp.cos(c_sym[8]/2 - c_sym[2])
        theta_3 = c_sym[5] - c_sym[2]
        
        self.FKine =  sp.Matrix([[x_c], [y_c], [theta_c], [phi1], [phi2], [phi3], [p], [q], [B]])
        self.IKine = sp.Matrix([[x_1], [y_1], [theta_1], [x_2], [y_2], [theta_2], [x_3], [y_3], [theta_3]])
        self.Jacob = self.FKine.jacobian(r_sym)
        self.JacobInv = self.IKine.jacobian(c_sym)

        self.FKine_func = sp.lambdify(r_sym, self.FKine, 'numpy')
        self.IKine_func = sp.lambdify(c_sym, self.IKine, 'numpy')
        self.Jacobian_func = sp.lambdify(r_sym, self.Jacob, 'numpy')
        self.JacobianInv_func = sp.lambdify(c_sym, self.JacobInv, 'numpy')
        self.cluster_angle_index = [2, 3, 4, 5, 8]

    #For testing cluster:
    def get_desired_position(self):
        r = np.array(self.IKine_func(*self.c_des.flatten())).astype(np.float64)
        return r

    def test_transforms(self, r):
        c = np.array(self.FKine_func(*r.flatten())).astype(np.float64)
        r = np.array(self.IKine_func(*c.flatten())).astype(np.float64)
        return c, r


if __name__ == "__main__":
    # Example usage
    cluster = Cluster(numRobots=3, clusterType=ClusterConfig.TRICEN, clusterParams=[10, 10, math.pi/3])
    with open('cluster_expressions/FKine.txt', 'w') as f:
        f.write(sp.pretty(cluster.FKine, wrap_line=False))
    with open('cluster_expressions/IKine.txt', 'w') as f:
        f.write(sp.pretty(cluster.IKine, wrap_line=False))
    with open('cluster_expressions/Jacob.txt', 'w') as f:
        f.write(sp.pretty(cluster.Jacob, wrap_line=False))
    with open('cluster_expressions/JacobInv.txt', 'w') as f:
        f.write(sp.pretty(cluster.JacobInv, wrap_line=False))