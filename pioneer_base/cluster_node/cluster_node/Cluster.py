
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
    def __init__(self, numRobots=3, clusterType=ClusterConfig.TRIANGLE, clusterParams=[20, 20, math.pi/2], KPgains=None, KVgains=None):
        #cluster control variables DOF which is assumed 3
        self.cdes = np.zeros((numRobots*3, 1))
        #self.cdes[0:2] = np.array([10, 10]).reshape(2, 1)
        self.cdes[(numRobots-1)*3:(numRobots)*3] = np.reshape(clusterParams, (numRobots, 1))
        self.cddes = np.zeros((numRobots*3, 1)) 
        self.cdddes = np.zeros((numRobots*3, 1)) 

        #Symbolic kinematic transform vars
        self.FKine, self.IKine, self.Jacob, self.JacobInv = None, None, None, None
        self.configureCluster(numRobots, clusterType)
        
        try:
            assert KPgains is not None and len(KPgains) == numRobots*3, "KPgains must be a list of length numRobots*3"
            assert KVgains is not None and len(KVgains) == numRobots*3, "KVgains must be a list of length numRobots*3"
        except AssertionError as e:
            print(f"AssertionError: {e}")
            return

        self.Kp =  np.diag(KPgains) #nm*nm diagonal matrix of gains
        self.Kv =  np.diag(KVgains) #nm*nm diagonal matrix of gains
        self.c = np.zeros((numRobots*3, 1))  # Cluster actual position
        self.cd = np.zeros((numRobots*3, 1))  # Cluster actual velocity


    def getVelocityCommand(self, r, rd):
        self.updateClusterPosition(r, rd)
        cd_cmd = self.calculateLinearControl()
    
        c_sym = sp.symbols('c0:9')
        subs_dict = {c_sym[i]: self.c[i, 0] for i in range(len(c_sym))} #map symbols to values
        rd = np.dot(np.array(self.JacobInv.subs(subs_dict)), cd_cmd)

        return rd

    def calculateLinearControl(self):
        return self.cdddes + np.dot(self.Kv, (self.cddes - self.cd)) + np.dot(self.Kp, (self.cdes - self.c))

    def getDesiredClusterPosition(self):
        c_sym = sp.symbols('c0:9')
        subs_dict = {c_sym[i]: self.cdes[i, 0] for i in range(len(c_sym))} #map symbols to values
        rd = np.array(self.IKine.subs(subs_dict).evalf()).astype(np.float64)
        return rd
    
    def getPositionalError(self):
        c_err = self.cdes - self.c
        c_sym = sp.symbols('c0:9')
        subs_dict = {c_sym[i]: c_err[i, 0] for i in range(len(c_sym))} #map symbols to values
        rd = np.array(self.IKine.subs(subs_dict).evalf()).astype(np.float64)
        return rd

    def testTransforms(self, r):
        r_sym = sp.symbols('r0:9')
        subs_dict = {r_sym[i]: r[i, 0] for i in range(len(r_sym))} #map symbols to values
        subs_dict[sp.symbols('0c')] = 0 #pass in dummy

        c = np.array(self.FKine.subs(subs_dict).evalf()).astype(np.float64)
        
        c_sym = sp.symbols('c0:9')
        subs_dict = {c_sym[i]: c[i, 0] for i in range(len(c_sym))} #map symbols to values
        r = np.array(self.IKine.subs(subs_dict).evalf()).astype(np.float64)

        return c, r

    def updateClusterPosition(self, r, rd):
        r_sym = sp.symbols('r0:9')
        subs_dict = {r_sym[i]: r[i, 0] for i in range(len(r_sym))} #map symbols to values
        subs_dict[sp.symbols('0c')] = self.cdes[8, 0] #pass in cluster heading

        self.c = np.array(self.FKine.subs(subs_dict).evalf()).astype(np.float64)
        self.cd = np.dot(np.array(self.Jacob.subs(subs_dict).evalf()).astype(np.float64), rd)


    #Based on the given cluster configuration will set the symbolic kinematic transform equations
    #from a set of prederived transforms
    def configureCluster(self, robots, clusterType):
        
        #Triangle configuration with cluster at centroid
        if(robots==3 and clusterType == ClusterConfig.TRIANGLE):
            r_sym = sp.symbols('r0:9') #symbols for robot space state variables
            theta_C = sp.symbols('0c') #symbol for cluster heading

            #Derived FKine equations for cluster space configuration
            x_c = (r_sym[0] + r_sym[3] + r_sym[6]) / 3
            y_c = (r_sym[1] + r_sym[4] + r_sym[7]) / 3
            theta_c = sp.atan2(2 / 3 * r_sym[0] - 1 / 3 * (r_sym[3] - r_sym[6]), 2 / 3 * r_sym[1] - 1 / 3 * (r_sym[4] + r_sym[7]))
            phi1 = r_sym[2] + theta_C
            phi2 = r_sym[5] + theta_C
            phi3 = r_sym[8] + theta_C
            p = sp.sqrt((r_sym[0]-r_sym[3])**2 + (r_sym[1]-r_sym[4])**2)
            q = sp.sqrt((r_sym[6]-r_sym[0])**2 + (r_sym[1]-r_sym[7])**2)
            B = sp.acos((p**2 + q**2 - (r_sym[6]-r_sym[3])**2 - (r_sym[7]-r_sym[4])**2)/(2*p*q))

            self.FKine =  sp.Matrix([[x_c], [y_c], [theta_c], [phi1], [phi2], [phi3], [p], [q], [B]])
            #print("Forward Kinematics Equations:")
            #print(self.FKine)

            c_sym = sp.symbols('c0:9') #symbols for cluster space state variables
            r = sp.sqrt((c_sym[7]+c_sym[6]*sp.cos(c_sym[8]))**2 + (c_sym[6]*sp.sin(c_sym[8]))**2)
            #Derived IKine equations for cluster space configuration
            x_1 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2])
            y_1 = c_sym[1] - 1/3 * r * sp.cos(c_sym[2])
            theta_1 = c_sym[3] - c_sym[2]
            x_2 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2]) - c_sym[6] * sp.sin(c_sym[8]/2 + c_sym[2])
            y_2 = c_sym[1] - 1/3 * r * sp.cos(c_sym[2]) - c_sym[6] * sp.cos(c_sym[8]/2 + c_sym[2])
            theta_2 = c_sym[4] - c_sym[2]
            x_3 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2]) + c_sym[7] * sp.sin(c_sym[8]/2 + c_sym[2])
            y_3 = c_sym[1] - 1/3 * r * sp.cos(c_sym[2]) - c_sym[7] * sp.cos(c_sym[8]/2 + c_sym[2])
            theta_3 = c_sym[5] - c_sym[2]
            self.IKine = sp.Matrix([[x_1], [y_1], [theta_1], [x_2], [y_2], [theta_2], [x_3], [y_3], [theta_3]])
            #print("Inverse Kinematics Equations:")
            #print(self.IKine)

            self.Jacob = self.FKine.jacobian(r_sym)
            self.JacobInv = self.IKine.jacobian(c_sym)
    
        
