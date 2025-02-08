import numpy as np
import math
import sympy as sp 
from enum import Enum

class ClusterConfig(Enum):
    TRIANGLE = "Triangle"
    ELLIPSE = "Ellipse"
    DIAMOND = "Diamond"
    SQUARE = "Square"
    LINE = "Line"
    
    def __str__(self):
        return self.value

class Cluster():
    #Create a cluster with a set number of robots in a specific configuration with initial parameters
    def __init__(self, numRobots=3, clusterType=ClusterConfig.TRIANGLE, clusterParams=[3, 3, math.pi/3]):
        #cluster control variables DOF which is assumed 3
        self.robots = numRobots
        self.cdes = np.zeros((robots*3, 1))  
        self.cddes = np.zeros((robots*3, 1)) 
        self.cdddes = np.zeros((robots*3, 1)) 

        self.FKine = self.configureCluster()
        self.Fm = np.zeros((9, 1))
        self.Kp = 4 #nm*nm diagonal matrix of gains
        self.Kv = 3 #nm*nm diagonal matrix of gains
        self.c = np.zeros((9, 1))  # Cluster actual position
        self.cd = np.zeros((9, 1))  # Cluster actual velocity
        self.r = np.zeros((9, 1))  #Latest robot positions
        self.rd = np.zeros((9, 1)) # Latest robot velocities

    def computeRobotForce(self):
        calculateLinearControl()
        #cluster EOM 
        #Jacobian tronspose to convert to robot force vector

    def calculateLinearControl(self):
        #compute cluster 
        updateClusterPosition()
        updateClusterVelocity()
        self.Fm = self.cdddes + self.Kv*(self.cddes - self.cd) + self.Kp(self.cdes - self.c)

    def computeJacobian(self):
        r_symbols = sp.symbols('r0:9')

        # Define the functions based on the elements of r
        f1 = (r_symbols[0] + r_symbols[3] + r_symbols[6]) / 3
        print(f1)
        f2 = (r_symbols[1] + r_symbols[4] + r_symbols[7]) / 3
        f3 = sp.atan2(2 * r_symbols[0] - r_symbols[3] - r_symbols[6], 2 * r_symbols[1] - r_symbols[4] - r_symbols[7])
        f4 = r_symbols[2] - self.c[3]
        f5 = r_symbols[5] - self.c[3]
        f6 = r_symbols[8] - self.c[3]
        f7 = sp.sqrt(r_symbols[0]**2 + r_symbols[1]**2)
        f8 = 3
        f9 = sp.pi / 3

        # Create a vector of functions
        F = sp.Matrix([[f1], [f2], [f3], [f4], [f5], [f6], [f7], [f8], [f9]])

        # Create a vector of variables
        R = sp.Matrix(r_symbols)

        # Compute the Jacobian matrix
        J = F.jacobian(R)

        subs_dict = {r_symbols[i]: self.r[i, 0] for i in range(len(r_symbols))}
        print(subs_dict)
        # Substitute the values into the Jacobian
        J_subs = J.subs(subs_dict)

        return J_subs

    #based on FKine
    def updateClusterPosition():
        varLock = true #Lock vars from being modified during computation
        self.c = np.array([
            [(r[0]+r[3]+r[6])/3], 
            [(r[1]+r[4]+r[7])/3], 
            [math.atan2(2*r[0]-r[3]-r[6], 2*r[1]-r[4]-r[7])], 
            [r[2]-c[3]], 
            [r[5]-c[3]], 
            [r[8]-c[3]], 
            [math.sqrt(math.pow(r[0]-r[3]), 2), math.pow((r[0]-r[3]), 2)], 
            [3], 
            [math.pi/3]])
        return c
    def updateClusterVelocity():
        varLock = true
        Jacobian = computeJacobian()
        self.cd = Jacobian * self.rd

    def updateRobotPosition(self, pos):
        self.r = pos

    def updateRobotVelocity(self, vel):
        self.rd = vel
    
    #Determine cluster space state variable by choosing from a list of pre derived cluster configurations
    def configureCluster(self, clusterType, clusterParams):
        
        #Triangle config
        if(self.numRobots==3 and clusterType == ClusterConfig.Triangle):
            r_sym = sp.symbols('r0:9') #symbols for robot space state variables
            theta_C = sp.symbols('0c') #symbol for cluster heading
            #Derived FKine equations for cluster space configuration
            f1 = (r_sym[0] + r_sym[3] + r_sym[6]) / 3
            f2 = (r_sym[1] + r_sym[4] + r_sym[7]) / 3
            f3 = sp.atan2((2 / 3 * r_sym[0] - 1 / 3 (r_sym[3] - r_sym[6]))/(2 / 3 * r_symbols[1] - 1 / 3 (r_symbols[4] + r_symbols[7])))
            f4 = r_sym[2] + theta_C
            f5 = r_sym[5] + theta_C
            f6 = r_sym[8] + theta_C
            f7 = sp.sqrt((r_sym[0]-r_sym[3])**2 + (r_sym[1]-r_sym[4])**2)
            f8 = sp.sqrt((r_sym[6]-r_sym[0])**2 + (r_sym[1]-r_sym[7])**2)
            f9 = sp.acos((f7**2 + f8**2 - (r_sym[6]-r_sym[3])**2 - (r_sym[7]-r_sym[4])**2)/(2*f7*f8))

            self.FKine =  sp.Matrix([[f1], [f2], [f3], [f4], [f5], [f6], [f7], [f8], [f9]])
        
            q = sp.symbols('q0:9')  # Cluster space variables corresponding to FKine

            # Solve for r_sym in terms of q (Inverse Kinematics)
            IKine = sp.solve(FKine - sp.Matrix(q), r_sym)
            print("Inverse Kinematics Solutions:")
            for key, value in IKine.items():
                print(f"{key} = {value}")
    #checks to see if robot is already in cluster and if not adds it
    """ def checkRobotID(self, id):
        incluster = false
        for robot in robots:
            if id==robot:
                incluster = True
        if not incluster:
            robots.append(id)
            configureCluster() """

        
