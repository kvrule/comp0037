import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.robotBooting = True # Necessary flag so it can choose a destination by updating frontiers.
        self.robotX = 0
        self.robotY = 0

        rospy.Subscriber("robot0/odom", Odometry, self.updatePose)

        self.blackList = []
        self.frontiers = []

    def updatePose(self, odomData):
        """ Updates the x and y position of the robot in terms of cell coordinates """

        pose = odomData.pose.pose.position
        robotCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((pose.x, pose.y))
        self.robotX = robotCoords[0]
        self.robotY = robotCoords[1]

    def updateFrontiers(self):
        """ Updates the frontiers based on the most recent version of the
            occupancy grid. """

        newFrontiers = []

        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)

                if self.isFrontierCell(x, y) is True:
                    newFrontiers.append(candidate)
        
        self.frontiers = newFrontiers

    def _getDistanceFromRobotToFrontier(self, frontierX, frontierY):
        return (self.robotX - frontierX) ** 2 + (self.robotY - frontierY) ** 2

    def chooseNewDestination(self):
        """ Chooses the next cell to go to based on the cell that is considered
            part of a frontier and that is the closest to the robot currently. """

        candidateGood = False
        destination = None
        smallestD2 = float('inf')

        if self.robotBooting:
            self.updateFrontiers()
            self.robotBooting = False

        for candidate in self.frontiers:
            candidateGood = True
            for k in range(0, len(self.blackList)):
                if self.blackList[k] == candidate:
                    candidateGood = False
                    break
            
            if candidate[0] == self.robotX and candidate[1] == self.robotY:
                candidateGood = False

            if candidateGood is True:
                d2 = self._getDistanceFromRobotToFrontier(candidate[0], candidate[1])

                if (d2 < smallestD2):
                    destination = candidate
                    smallestD2 = d2
        
        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
