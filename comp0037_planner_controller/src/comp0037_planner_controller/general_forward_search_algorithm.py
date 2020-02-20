# -*- coding: utf-8 -*-

from occupancy_grid import OccupancyGrid
from search_grid import SearchGrid
from planner_base import PlannerBase
from collections import deque
from cell import *
from planned_path import PlannedPath
from path_metric import PathMetric
from math import *
import rospy

class GeneralForwardSearchAlgorithm(PlannerBase):

    # This class implements the basic framework for LaValle's general
    # template for forward search. It includes a lot of methods for
    # managing the graphical output as well.
    
    def __init__(self, title, occupancyGrid):
        PlannerBase.__init__(self, title, occupancyGrid)
         
        # Flag to store if the last plan was successful
        self.goalReached = None

        # Keeps track of some key path planning metrics
        self.pathMetric = PathMetric()

    # These methods manage the queue of cells to be visied.
    def pushCellOntoQueue(self, cell):
        raise NotImplementedError()

    # This method returns a boolean - true if the queue is empty,
    # false if it still has some cells on it.
    def isQueueEmpty(self):
        raise NotImplementedError()

    # This method finds the first cell (at the head of the queue),
    # removes it from the queue, and returns it.
    def popCellFromQueue(self):
        raise NotImplementedError()

    # This method determines if the goal has been reached.
    def hasGoalBeenReached(self, cell):
        raise NotImplementedError()

    # This method gets the list of cells which could be visited next.
    def getNextSetOfCellsToBeVisited(self, cell):
        raise NotImplementedError()

    # This method determines whether a cell has been visited already.
    def hasCellBeenVisitedAlready(self, cell):
        raise NotImplementedError()

    def markCellAsVisitedAndRecordParent(self, cell, parentCell):
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell
        self.pathMetric.incrementCellsVisited()

    # Mark that a cell is dead. A dead cell is one in which all of its
    # immediate neighbours have been visited.
    def markCellAsDead(self, cell):
        cell.label = CellLabel.DEAD
    
    # Handle the case that a cell has been visited already.
    def resolveDuplicate(self, cell):
        raise NotImplementedError()
    
    # Compute the additive cost of performing a step from the parent to the
    # current cell. This calculation is carried out the same way no matter
    # what heuristics, etc. are used. The cost computed here takes account
    # of the terrain traversability cost using an equation a bit like that
    # presented in the lectures.
    def computeLStageAdditiveCost(self, parentCell, cell):       
        # If the parent is empty, this is the start of the path and the
        # cost is 0.
        if (parentCell is None):
            return 0
        
        # Travel cost is Cartesian distance
        dX = cell.coords[0] - parentCell.coords[0]
        dY = cell.coords[1] - parentCell.coords[1]
        # Terrain cost 
        #  Run this in matlab to visualise ro check the image
        # However, basically it builds up extremely quickly
        # x=[1:0.01:2];
        # c=min(1+(.2./((1.7-x).^2)).^2,1000);       
        cost=min(1+(0.2/((1.75-cell.terrainCost)**2))**2, 1000)
        L = sqrt(dX * dX + dY * dY)*cost# Multiplied by the terrain cost of the cell
        
        return L
        
    # The main search routine. The routine searches for a path between a given
    # set of coordinates. These are then converted into start and destination
    # cells in the search grid and the search algorithm is then run.
    def search(self, startCoords, goalCoords):

        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()
        
        # Keeps track of the queue size for updating path metrics. Initialised
        # to zero at the beginning of the search.
        queueSize = 0

        # Create or update the search grid from the occupancy grid and seed
        # unvisited and occupied cells.
        if (self.searchGrid is None):
            self.searchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid)
        else:
            self.searchGrid.updateFromOccupancyGrid()

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)
        self.start.setAlgoPriorityType(self.getAlgorithmType())
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        # Get the goal cell object and label it.
        self.goal = self.searchGrid.getCellFromCoords(goalCoords)
        self.goal.setAlgoPriorityType(self.getAlgorithmType())
        self.goal.label = CellLabel.GOAL

        # If the node is being shut down, bail out here.
        if rospy.is_shutdown():
            return False

        # Draw the initial state
        self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)
        queueSize += 1
        self.pathMetric.updateMaxQueueLength(queueSize)

        # Reset the count
        self.numberOfCellsVisited = 0

        # Indicates if we reached the goal or not
        self.goalReached = False

        # Iterate until we have run out of live cells to try or we reached the goal.
        # This is the main computational loop and is the implementation of
        # LaValle's pseudocode
        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging.
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromQueue()
            queueSize -= 1
            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                break
            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
                    self.markCellAsVisitedAndRecordParent(nextCell, cell)
                    nextCell.updateDistanceFromGoal(goalCoords)
                    heuristic = self.calculateHeuristic("octile", nextCell.coords, goalCoords)
                    nextCell.setEstimatedDistanceFromGoal(heuristic)
                    nextCell.setPathCost(cell.pathCost + self.computeLStageAdditiveCost(cell, nextCell))
                    nextCell.setAlgoPriorityType(self.getAlgorithmType())
                    self.pushCellOntoQueue(nextCell)
                    queueSize += 1
                    self.pathMetric.updateMaxQueueLength(queueSize)
                    self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                else:
                    self.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()
        
        print "numberOfCellsVisited = " + str(self.numberOfCellsVisited)
        
        if self.goalReached:
            print "Goal reached"
        else:
            print "Goal not reached"

        return self.goalReached

    # Checks if the next cell in the path changes direction from the
    # current cell's path direction.
    def changingDirection(self, prevCell, currCell, nextCell):
        
        currXDiff = currCell.coords[0] - prevCell.coords[0]
        currYDiff = currCell.coords[1] - prevCell.coords[1]
        nextXDiff = nextCell.coords[0] - currCell.coords[0]
        nextYDiff = nextCell.coords[1] - currCell.coords[1]

        if currXDiff == nextXDiff and currYDiff == nextYDiff:
            return True
        else:
            return False

    # Converts the path of waypoints that move in the same direction
    # to just two points, a start and end point, rather than having
    # multiple points inbetween.
    def optimisePath(self, path):
        
        waypointsSize = len(path.waypoints)

        if waypointsSize == 0 or waypointsSize == 1:
            return path

        newWaypoints = []
        newWaypoints.append(path.waypoints[0]) # Add first cell.

        prevCell = path.waypoints[0]

        for i in range(1, waypointsSize - 1):
            currCell = path.waypoints[i]
            nextCell = path.waypoints[i + 1]

            if not self.changingDirection(prevCell, currCell, nextCell):
                newWaypoints.append(currCell)
            
            prevCell = currCell
        
        newWaypoints.append(path.waypoints[waypointsSize - 1])

        path.waypoints = newWaypoints
        return path

    # This method extracts a path from the pathEndCell to the start
    # cell. The path is a list actually sorted in the order:
    # cell(x_1), cell(x_2), ... , cell(x_K), cell(x_G). You can use
    # this method to try to find the path from any end cell. However,
    # depending upon the planner used, the results might not be
    # valid. In this case, the path will probably not terminate at the
    # start cell.
    def extractPathEndingAtCell(self, pathEndCell, colour):

        # Construct the path object and mark if the goal was reached
        path = PlannedPath()
        
        path.goalReached = self.goalReached
        
        # Initial condition - the goal cell
        path.waypoints.append(pathEndCell)
               
        # Start at the goal and find the parent. Find the cost associated with the parent
        cell = pathEndCell.parent
        path.travelCost = self.computeLStageAdditiveCost(pathEndCell.parent, pathEndCell)

        if cell is not None:
            prevDirecVector = [pathEndCell.coords[0] - cell.coords[0], pathEndCell.coords[1] - cell.coords[1]]
        else:
            prevDirecVector = [0, 0]
            cell = pathEndCell
        oldCell = pathEndCell
        
        # Iterate back through and extract each parent in turn and add
        # it to the path. To work out the travel length along the
        # path, you'll also have to add self at self stage.
        while (cell is not None):
            currDirecVector = [oldCell.coords[0] - cell.coords[0], oldCell.coords[1] - cell.coords[1]]
            self.pathMetric.calculateAndAddTotalTurnAngle(prevDirecVector, currDirecVector)
            prevDirecVector = currDirecVector
            oldCell = cell
            path.waypoints.appendleft(cell)
            path.travelCost = path.travelCost + self.computeLStageAdditiveCost(cell.parent, cell)
            cell = cell.parent
            
        path = self.optimisePath(path)

        # Update the stats on the size of the path
        path.numberOfWaypoints = len(path.waypoints)

        # Note that if we failed to reach the goal, the above mechanism computes a path length of 0.
        # Therefore, if we didn't reach the goal, change it to infinity
        if path.goalReached is False:
            path.travelCost = float("inf")

        self.pathMetric.setTotalTravelCost(path.travelCost)

        print "Path travel cost = " + str(path.travelCost)
        print "Path cardinality = " + str(path.numberOfWaypoints)

        self.pathMetric.printMetrics()
        
        # Draw the path if requested
        if (self.showGraphics == True):
            self.plannerDrawer.update()
            self.plannerDrawer.drawPathGraphicsWithCustomColour(path, colour)
            self.plannerDrawer.waitForKeyPress()
        
        # Return the path
        return path

    # Extract the path from a specified end cell to the start. This is not
    # necessarily the full path. Rather, it lets us illustrate parts of the
    # path.
    def extractPathEndingAtCoord(self, endCellCoord):
        endCell = self.searchGrid.getCellFromCoords(endCellCoord)
        self.extractPathEndingAtCell(endCell, 'red')

    # Extract the path between the start and goal.
    def extractPathToGoal(self):
        path = self.extractPathEndingAtCell(self.goal, 'yellow')
       
        return path
            
