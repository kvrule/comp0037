# -*- coding: utf-8 -*-

from math import sqrt
from cell_based_forward_search import CellBasedForwardSearch
try:
    from queue import PriorityQueue # For Python 3
except:
    from Queue import PriorityQueue # For Python 2.7

# This class implements the Breadth first search - Dijkstra planning
# algorithm. It works by using a priority queue: cells are pushed
# onto the queue, and are popped based on prioritising cells with
# the smallest path cost from the previous cell.

class AStarPlanner(CellBasedForwardSearch):

    """ Implements Dijsktra's algorithm which prioritises cells with the
        smallest path cost from the previous cell and updates the lengths
        to its neighbours. """

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = PriorityQueue()
        self.ALGORITHM_TYPE = "A*"
        self.HEURISTIC_CONSTANT = 0

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.priorityQueue.put(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.priorityQueue.empty()

    # Pull the highest priority cell.
    def popCellFromQueue(self):
        cell = self.priorityQueue.get()
        return cell

    def _updatePriorityOfCellInQueue(self, cell):
        tempCellsList = []
        
        nextCell = self.priorityQueue.get()
        while (not (nextCell.coords[0] == cell.coords[0] and 
                    nextCell.coords[1] == cell.coords[1])):
            tempCellsList.append(nextCell)
            nextCell = self.priorityQueue.get()

        self.priorityQueue.put(cell) # Puts new updated cell back in to queue
        
        for e in tempCellsList:
            self.priorityQueue.put(e)

    def resolveDuplicate(self, cell, parentCell):
        new_path_cost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell, cell)
        
        if new_path_cost < cell.pathCost:
            cell.parent = parentCell
            cell.pathCost = new_path_cost

            self._updatePriorityOfCellInQueue(cell)

    def getAlgorithmType(self):
        return self.ALGORITHM_TYPE

    def calculateHeuristic(self, heuristic, startCoords, goalCoords):
        if heuristic == "euclidean":
            xDiff = startCoords[0] - goalCoords[0]
            yDiff = startCoords[1] - goalCoords[1]
            distanceFromGoal = sqrt((xDiff ** 2) + (yDiff ** 2))
        elif heuristic == "octile":
            xDiff = abs(startCoords[0] - goalCoords[0])
            yDiff = abs(startCoords[1] - goalCoords[1])
            distanceFromGoal = max(xDiff, yDiff) + (sqrt(2) - 1) * min(xDiff, yDiff)
        elif heuristic == "manhattan":
            xDiff = abs(startCoords[0] - goalCoords[0])
            yDiff = abs(startCoords[1] - goalCoords[1])
            distanceFromGoal = xDiff + yDiff
        else:
            distanceFromGoal = self.HEURISTIC_CONSTANT
        
        return distanceFromGoal