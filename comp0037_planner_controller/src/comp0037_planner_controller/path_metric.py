# -*- coding: utf-8 -*-

from math import sqrt
from math import acos

class PathMetric(object):

    """ Keeps track of path planning metrics and provides a method, printMetrics,
        to display all metrics """

    def __init__(self):

        # Initialise metrics to zero
        self._numCellsVisited = 0
        self._maxQueueLength = 0
        self._totalTravelCost = 0
        self._totalTurnAngle = 0

    def incrementCellsVisited(self):
        """ Increments the number of cells visited metric """

        self._numCellsVisited += 1

    def updateMaxQueueLength(self, length):
        """ Updates max queue length metric to the largest value between length
            and the current max queue length metric. """
        
        self._maxQueueLength = max(self._maxQueueLength, length)

    def setTotalTravelCost(self, cost):
        """ Sets total travel cost metric to cost. """

        self._totalTravelCost = cost
    
    def calculateAndAddTotalTurnAngle(self, prevVec, currVec):
        """ Calculates the turn angle using the dot product of the direction
            vector to the previous cell and the direction vector to the
            current cell and adds it to the current total turn angle. """
        
        dotProduct = prevVec[0] * currVec[0] + prevVec[1] * currVec[1]
        prevSize = sqrt((prevVec[0] ** 2) + (prevVec[1] ** 2))
        currSize = sqrt((currVec[0] ** 2) + (currVec[1] ** 2))

        theta = abs(acos(dotProduct / (prevSize * currSize)))
        theta = theta * (180 / 3.1415) # Converts from radians to degrees
        self._totalTurnAngle += theta

    def printMetrics(self):
        """ Prints all path metrics, with their descriptions and corresponding
            values. """

        print("Number of cells visited: {}".format(self._numCellsVisited))
        print("Maximum length of queue: {}".format(self._maxQueueLength))
        print("Total travel cost of planned path: {}".format(self._totalTravelCost))
        print("Total angle turned in path: {}".format(self._totalTurnAngle))