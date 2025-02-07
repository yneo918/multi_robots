import numpy as np
import math
import sympy as sp 

class Cluster():
    def __init__(self):
        #cluster control variables where n is the number of robots in the cluster and m is their DOF
        self.cdes = np.array([[5], [4], [math.pi/2], [0], [0], [0], [3], [3], [math.pi/3]])  # Define as vertical matrix in one line
        self.cddes = np.zeros((9, 1))  # Cluster ideal velocities 
        self.cdddes = np.zeros((9, 1))  # Cluster ideal accelerations likely ignore for now
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
        
    #checks to see if robot is already in cluster and if not adds it
    """ def checkRobotID(self, id):
        incluster = false
        for robot in robots:
            if id==robot:
                incluster = True
        if not incluster:
            robots.append(id)
            configureCluster() """

        
