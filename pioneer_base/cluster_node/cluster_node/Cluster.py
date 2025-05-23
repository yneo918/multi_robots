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
    num_robots: Number of robots in the cluster
    cluster_type: The geometric shape representing configuration of the cluster (TRICEN is the only one implemented currently)
    cluster_params: array of cluster configuration parameters dependent on configuration (for TRICEN it is [p, q, B] which is the SSA of the triangle)
    KPgains: Proportional gains for the cluster control (must be length num_robots*DOF where DOF is 3 for land rovers)
    KVgains: Derivative gains for the cluster control (must be length num_robots*DOF where DOF is 3 for land rovers)
    control_mode: The control mode for the cluster (POS or VEL)
"""

ROVER_DOF = 3

class Cluster():
    #Create a cluster with a set number of robots in a specific configuration with initial parameters
    def __init__(self, num_robots=3, cluster_type=ClusterConfig.TRICEN, KPgains=None, KVgains=None, control_mode="POS"):
        self.num_robots = num_robots
        self.cluster_type = cluster_type
        if control_mode not in ["POS", "VEL"]:
            raise ValueError("control_mode must be either 'POS' or 'VEL'")
        self.control_mode = control_mode
        try:
            if KPgains is None:
                KPgains = [1.0] * (self.num_robots*ROVER_DOF)
            if KVgains is None:
                KVgains = [1.0] * (self.num_robots*ROVER_DOF)
            assert KPgains is not None and len(KPgains) == self.num_robots*ROVER_DOF, "KPgains must be a list of length self.num_robots*ROVER_DOF"
            assert KVgains is not None and len(KVgains) == self.num_robots*ROVER_DOF, "KVgains must be a list of length self.num_robots*ROVER_DOF"
        except AssertionError as e:
            print(f"AssertionError: {e}")
            return
        self.Kp = np.diag(KPgains) #nm*nm diagonal matrix of gains
        self.Kv = np.diag(KVgains) #nm*nm diagonal matrix of gains
        self.cluster_angle_index = []
        self.initialize_cluster()
    
    def initialize_cluster(self):
        self.configure_cluster(self.num_robots, self.cluster_type)

    #Given the current robot state space variables and desired cluster state space variables, calculate robot velocity vector
    def get_velocity_command(self, r, rdot, c_des, cdot_des):
        c, cdot = self.calculate_cluster_position(r, rdot)
        cdot_cmd = self.calculate_linear_control(c, cdot, c_des, cdot_des)
        _rdot = np.dot(np.array(self.JacobianInv_func(*c.flatten())), cdot_cmd)
        return cdot_cmd, _rdot, c

    #Given a the current robot state space variables, update the cluster state space variables
    def calculate_cluster_position(self, r, rdot):
        c = self.FKine_func(*r.flatten())
        for i in self.cluster_angle_index:
            c[i, 0] = self.wrap_to_pi(c[i, 0])
        cdot = np.dot(np.array(self.Jacobian_func(*r.flatten())).astype(np.float64), rdot)
        return c, cdot

    #Linear control equation 
    def calculate_linear_control(self, c, cdot, c_des, cdot_des):
        if self.control_mode == "VEL":
            return cdot_des
        if self.control_mode == "POS":
            c_diff = c_des - c
            for i in self.cluster_angle_index:
                c_diff[i, 0] = self.wrap_to_pi(c_diff[i, 0])
            cdot_cmd = np.dot(self.Kp, c_diff)
            return cdot_cmd
        else:
            raise ValueError("control_mode must be either 'POS' or 'VEL'")
    def wrap_to_pi(self, t):
        return (t + np.pi) % (2 * np.pi) - np.pi

    #Based on the given cluster configuration will set the symbolic kinematic transform equations
    #from a set of prederived transforms
    def configure_cluster(self, robots, cluster_type):
        if robots == 3:
            if cluster_type == ClusterConfig.TRICEN:
                self.cluster_config_tricen()
        if robots == 5:
            if cluster_type == ClusterConfig.TRILEAD:
                self.cluster_config_trilead()


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
    

    def cluster_config_trilead(self):
        r_sym = sp.symbols('r0:15') #symbols for robot space state variables
        c_sym = sp.symbols('c0:15') #symbols for cluster space state variables

        # cluster space configuration by Robot space variables
        x_c = r_sym[0]
        y_c = r_sym[1]
        
        theta_c = sp.atan2(r_sym[1] - r_sym[4], r_sym[3] - r_sym[0])

        phi = [r_sym[2] - theta_c, r_sym[5] - theta_c, r_sym[8] - theta_c, r_sym[11] - theta_c, r_sym[14] - theta_c]
        d = [0, 
             sp.sqrt((r_sym[0] - r_sym[3])**2 + (r_sym[1] - r_sym[4])**2),
             sp.sqrt((r_sym[6] - r_sym[0])**2 + (r_sym[7] - r_sym[1])**2),
             sp.sqrt((r_sym[3] - r_sym[9])**2 + (r_sym[4] - r_sym[10])**2),
             sp.sqrt((r_sym[6] - r_sym[12])**2 + (r_sym[7] - r_sym[13])**2)]
        beta = [0, 0,
                sp.atan2(r_sym[7] - r_sym[1], r_sym[0] - r_sym[6]) - theta_c,
                sp.atan2(r_sym[9] - r_sym[3], r_sym[10] - r_sym[4]) - theta_c,
                sp.atan2(r_sym[12] - r_sym[6], r_sym[13] - r_sym[7]) - theta_c]

        # robot space configuration by cluster space variables
        x = [c_sym[0]]
        y = [c_sym[1]]
        theta = [c_sym[2] + c_sym[3]]

        x.append(x[0] + c_sym[8] * sp.cos(c_sym[2]))
        y.append(y[0] + c_sym[8] * -sp.sin(c_sym[2]))
        theta.append(c_sym[2] + c_sym[4])

        x.append(x[0] + c_sym[9] * -sp.cos(c_sym[12] + c_sym[2]))
        y.append(y[0] + c_sym[9] * sp.sin(c_sym[12] + c_sym[2]))
        theta.append(c_sym[2] + c_sym[5])

        x.append(x[1] + c_sym[10] * sp.sin(c_sym[13] + c_sym[2]))
        y.append(y[1] + c_sym[10] * sp.cos(c_sym[13] + c_sym[2]))
        theta.append(c_sym[2] + c_sym[6])

        x.append(x[2] + c_sym[11] * sp.sin(c_sym[14] + c_sym[2]))
        y.append(y[2] + c_sym[11] * sp.cos(c_sym[14] + c_sym[2]))
        theta.append(c_sym[2] + c_sym[7])

        self.FKine = sp.Matrix([x_c, y_c, theta_c] + phi + d[1:] + beta[2:])
        self.IKine = sp.Matrix([val for triplet in zip(x, y, theta) for val in triplet])
        self.Jacob = self.FKine.jacobian(r_sym)
        self.JacobInv = self.IKine.jacobian(c_sym)

        self.FKine_func = sp.lambdify(r_sym, self.FKine, 'numpy')
        self.IKine_func = sp.lambdify(c_sym, self.IKine, 'numpy')
        self.Jacobian_func = sp.lambdify(r_sym, self.Jacob, 'numpy')
        self.JacobianInv_func = sp.lambdify(c_sym, self.JacobInv, 'numpy')
        self.cluster_angle_index = [2, 3, 4, 5, 6, 7, 12, 13, 14]

    #For testing cluster:
    def get_desired_position(self, c_des):
        if self.IKine_func is None:
            return None
        r = np.array(self.IKine_func(*c_des.flatten())).astype(np.float64)
        return r

    def test_transforms(self, r):
        c = np.array(self.FKine_func(*r.flatten())).astype(np.float64)
        r = np.array(self.IKine_func(*c.flatten())).astype(np.float64)
        return c, r


if __name__ == "__main__":
    # Example usage
    cluster = Cluster(num_robots=5, cluster_type=ClusterConfig.TRILEAD)
    cluster.set_c_des([0,0,0,0,0,0,0,0,2,3,4,5,0,0.5,0])
    print("R",cluster.get_desired_position([0,0,0,0,0,0,0,0,2,3,4,5,0,0.5,0]))
    with open('cluster_expressions/FKine.txt', 'w') as f:
        f.write(sp.pretty(cluster.FKine, wrap_line=False))
    with open('cluster_expressions/IKine.txt', 'w') as f:
        f.write(sp.pretty(cluster.IKine, wrap_line=False))
    with open('cluster_expressions/Jacob.txt', 'w') as f:
        f.write(sp.pretty(cluster.Jacob, wrap_line=False))
    with open('cluster_expressions/JacobInv.txt', 'w') as f:
        f.write(sp.pretty(cluster.JacobInv, wrap_line=False))