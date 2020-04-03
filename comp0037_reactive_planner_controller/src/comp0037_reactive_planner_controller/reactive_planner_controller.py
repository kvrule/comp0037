# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.aisle import Aisle

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None

        # Map that has the aisle as the key and the intermediate coordinate as a value.
        self.aisleCoordsMap = self.getAisleIntermediateCoords()
        self.startPoint = self.controller.getCurrentPose()

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def getAisleIntermediateCoords(self):
        # Creates a mapping of the aisles to their intermediate coordinates.
        
        initialPoint = (28, 20)
        aisleMap = {} # Maps aisle value to the intermediate point.
        offset = 15 # Gap distance between each aisle.

        for aisle in Aisle:
            point = (initialPoint[0] + (aisle.value * offset), initialPoint[1])
            aisleMap[aisle] = point
        
        return aisleMap

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()

        pathBlocked = False
        
        waypointSize = len(self.currentPlannedPath.waypoints)
        for i in range(waypointSize):
            cell = self.currentPlannedPath.waypoints[i]

            # If getCell returns 1 it means there is an obstacle in that cell.
            if self.occupancyGrid.getCell(cell.coords[0], cell.coords[1]) == 1:
                pathBlocked = True
                break
        
        if pathBlocked:
            self.controller.stopDrivingToCurrentGoal()

    # Prints original path cost through aisle B and the alternate path cost.
    def printInitialPathCosts(self, originalPathCost, alternatePathCost):

        print("ORIGINAL PATH COST (AISLE B): {0}".format(originalPathCost))
        print("ALTERNATE PATH COST: {0}".format(alternatePathCost))

    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):

        p = 0.8 # Probability obstacle is present
        L_W = 2 # Wait cost constant

        aisleBPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, Aisle.B)
        alternateAisle = self.chooseAisle(startCellCoords, goalCellCoords)
        alternatePath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords,
                                                    alternateAisle)

        originalPathCost = len(aisleBPath.waypoints) + (p * L_W)
        alternatePathCost = len(alternatePath.waypoints)

        self.printInitialPathCosts(originalPathCost, alternatePathCost)

        # Choose the initial path immediately, based on estimated path costs.
        if originalPathCost < alternatePathCost: 
            return Aisle.B
        else: 
            return alternateAisle

    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.C

    # Prints the waiting plan and re-plan cost values cleanly on system out
    def printPathCost(self, waitingPlanCost, replanCost):

        print("WAITING PLAN COST: {0}".format(waitingPlanCost))
        print("RE-PLAN COST: {0}".format(replanCost))

    # Calculates the cost of the waiting plan.
    def calculateWaitPlanCost(self):

        L_W = 2 # Wait time cost defined in assignment as 2.

        # The length of the list of waypoints give the path cost of the route,
        # which is added to the wait time to give wait plan cost.
        return L_W + len(self.currentPlannedPath.waypoints)

    # Calculates the re-plan cost.
    def calculateReplanCost(self, pathCostToObstacle, startCoord, goalCoord):

        newAisle = self.chooseAisle(startCoord, goalCoord)
        newPath = self.planPathToGoalViaAisle(startCoord, goalCoord, newAisle)

        return len(newPath.waypoints) + pathCostToObstacle

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):

        pathCostToObstacle = 0 # Counts each waypoint until robot arrives at waiting region
        waitingPlanCost = 0 # Just to declare the value otherwise Python complains.
        replanCost = 0 # Just to declare the value otherwise Python complains.

        for waypoint in self.currentPlannedPath.waypoints:
            # Only consider cells in the path that are obstacle cells.
            if self.occupancyGrid.getCell(waypoint.coords[0], waypoint.coords[1]) == 1:
                waitingPlanCost = self.calculateWaitPlanCost()
                replanCost = self.calculateReplanCost(pathCostToObstacle, startCellCoords, goalCellCoords)

                if waitingPlanCost < replanCost:
                    self.printPathCost(waitingPlanCost, replanCost)

                    return True
            
            pathCostToObstacle += 1

        self.printPathCost(waitingPlanCost, replanCost)

        return False

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):

        continueWaiting = True

        while continueWaiting:
            obstacleFound = False

            for waypoint in self.currentPlannedPath.waypoints:
                if self.occupancyGrid.getCell(waypoint.coords[0], waypoint.coords[1]) == 1:
                    obstacleFound = True
                    break

            if not obstacleFound:
                continueWaiting = False
    
    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):

        # Note that, if the robot has waited, it might be tasked to drive down the
        # aisle it's currently on. Your code should handle this case.
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = aisle

        # Implement your method here to construct a path which will drive the robot
        # from the start to the goal via the aisle.
        intermediateCellCoords = self.aisleCoordsMap[aisle]
        pathToGoalFound = self.planner.search(startCellCoords, intermediateCellCoords)

        # If we can't reach the goal, give up and return
        if pathToGoalFound is False:
            rospy.logwarn("Could not find a path to the intermediate goal at (%d, %d)", \
                            intermediateCellCoords[0], intermediateCellCoords[1])
            return None
        
        currentPlannedPath = self.planner.extractPathToGoal() 

        # This path is currently the path to the intermediate cell.
        pathToGoalFound = self.planner.search(intermediateCellCoords, goalCellCoords)

        # If we can't reach the goal, give up and return
        if pathToGoalFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                            goalCellCoords[0], goalCellCoords[1])
            return None
        
        secondPlannedPath = self.planner.extractPathToGoal() 

        # Attach remaining path to the goal cell, to the path that goes from
        # start to intermediate cell.
        for waypoint in secondPlannedPath.waypoints:
            currentPlannedPath.waypoints.append(waypoint)

        return currentPlannedPath

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Work out the initial aisle to drive down
        aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        
        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.           
            
            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisleToDriveDown)
            self.gridUpdateLock.release()

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                return False

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords)

            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears()
            else:
                aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
