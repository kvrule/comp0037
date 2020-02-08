# -*- coding: utf-8 -*-

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

    def printMetrics(self):
        """ Prints all path metrics, with their descriptions and corresponding
            values. """

        print("Number of cells visited: {}".format(self._numCellsVisited))
        print("Maximum length of queue: {}".format(self._maxQueueLength))
        print("Total travel cost of planned path: {}".format(self._totalTravelCost))
        print("Total angle turned in path: {}".format(self._totalTurnAngle))