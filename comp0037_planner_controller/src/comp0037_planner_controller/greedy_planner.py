# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
try:
    from queue import PriorityQueue # For Python 3
except:
    from Queue import PriorityQueue # For Python 2.7

# This class implements the Best first search - Greedy planning
# algorithm. It works by using a priority queue: cells are pushed
# onto the queue, and are popped based on prioritising cells with
# the smallest euclidean distance to the goal node.

class GreedyPlanner(CellBasedForwardSearch):

    """ Provides a greedy 'best-first-search' algorithm that prioritises cells
        with the smallest Euclidean distance to the goal node. """

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = PriorityQueue()
        self.ALGORITHM_TYPE = "greedy"

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

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    def getAlgorithmType(self):
        return self.ALGORITHM_TYPE
    
    def calculateHeuristic(self, heuristic, startCoords, goalCoords):
        return