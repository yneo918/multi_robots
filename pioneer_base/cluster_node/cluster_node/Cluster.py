import math
class Cluster():
    def __init__(self):
        #cluster control variables
        self.cdes = np.array([[5], [4], [math.pi/2], [0], [0], [0], [3], [3], [math.pi/3]])  # Define as vertical matrix in one line
        self.cddes = np.zeros((9, 1))  # Cluster ideal velocities 
        self.cdddes = np.zeros((9, 1))  # Cluster ideal accelerations 
        self.Fm = np.zeros((9, 1))
        self.Kp = 4
        self.Kv = 3
        self.c = np.zeros((9, 1))  # Cluster actual position
        self.cd = np.zeros((9, 1))  # Cluster actual velocity
        self.r = np.zeros((9, 1))

    def calculateLinearControl():
        self.Fm = self.cdddes + self.Kv*(self.cddes - self.cd) + self.Kp(self.cdes - self.c)

    #based on FKine
    def updateClusterPosition()
        self.c = np.array([
            [(r[0]+r[3]+r[6])/3], 
            [(r[1]+r[4]+r[7])/3], 
            [math.atan2(2*r[0]-r[3]-r[6], 2*r[1]-r[4]-r[7])], 
            [r[2]-c[3]], 
            [r[5]-c[3]], 
            [r[8]-c[3]], 
            [math.sqrt(math.pow(r[0]-r[3]), 2), math.pow(r[0]-r[3]), 2)], 
            [3], 
            [math.pi/3]])
    #checks to see if robot is already in cluster and if not adds it
    def checkRobotID(self, id):
        incluster = false
        for robot in robots:
            if id==robot:
                incluster = True
        if not incluster:
            robots.append(id)
            configureCluster()

    #Recopngfigures clsuter based on number of robots in cluster 
    def configureCluster(self):

    def updateRobotHeading(id, heading):
        for robot in robots:
            if id==robot:
                cluster 

    def updateRobotPosition(id, heading):


    def calculateTrajectories():
        
