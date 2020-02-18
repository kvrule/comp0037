# -*- coding: utf-8 -*-

class RobotMetric(object):

    """ Keeps track of the robots physical metrics and provides a method, 
        printMetrics, to display all metrics """

    def __init__(self):

        # Initialise metrics to zero
        self._distanceTravelled = 0
        self._totalAngleTurned = 0
        self._totalTimeRequired = 0
    
    def addToDistanceTravelled(self, distance):
        """ Adds the distance argument to the current total distance travelled. """

        self._distanceTravelled += distance
    
    def addToTotalTurnAngle(self, angle):
        """ Adds the angle argument to the current total angle turned. """

        self._totalAngleTurned += abs(angle)
    
    def addTime(self):
        """ Adds 0.1 seconds to the time (0.1 is the chosen rospy rate in 
            the code). """
        
        self._totalTimeRequired += 0.1

    def _convertRadToDeg(self):
        return (180 * self._totalAngleTurned) / 3.1415

    def printMetrics(self):
        """ Prints all robot metrics, with their descriptions and corresponding
            values. """

        print("Robot metrics: -------------")
        print("Distance travelled: {}".format(self._distanceTravelled))
        print("Total angle turned: {}".format(self._convertRadToDeg()))
        print("Total time required: {} seconds".format(self._totalTimeRequired))