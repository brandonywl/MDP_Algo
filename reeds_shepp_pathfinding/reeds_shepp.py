import math
from utilities import utilities

#an action that can be taken in a Reeds-Shepp path
#these include moving forward/backward in a straight line and turning left/right in forwards/backwards direction
#constant turning radius is assumed, as well as constant velocity
class Action:
    # steering: 1 for left, -1 for right, 0 for straight
    # gear: 1 for forward, -1 for backward
    # value: if steering straight, this is the distance, else this is the angle of the arc in radians
    def __init__(self, steering, gear, value, startPos = None, endPos = None):
        self.steering = steering
        self.value = value
        self.gear = gear
        #self.isRadians = True
        self.streamlineTurning()
        self.startPos = startPos
        self.endPos = endPos
        self.arcCenter = None

    # if action is a turn, checks if value > pi, if so, reverse direction and turn 2*pi - value
    def streamlineTurning(self):
        if self.steering != 0 and self.value > math.pi:
            self.gear *= -1
            self.value = 2 * math.pi - self.value

    # convert value of action back to scale with the original turning radius, only for straight steering actions
    def convertValueOriginalScale(self, turningRadius):
        if self.steering == 0:
            self.value *= turningRadius

    """
    # convert value from radians to degrees for better readability
    def convertToDegrees(self):
        if self.steering != 0 and self.isRadians:
            self.value = math.degrees(self.value)
            if self.startPos:
                self.startPos = (self.startPos[0], self.startPos[1], math.degrees(self.startPos[2]))
                self.endPos = (self.startPos[0], self.startPos[1], math.degrees(self.endPos[2]))
            self.isRadians = False

    # convert value from degrees to radians
    def convertToRadians(self):
        if self.steering != 0 and not self.isRadians:
            self.value = math.radians(self.value)
            if self.startPos:
                self.startPos = (self.startPos[0], self.startPos[1], math.radians(self.startPos[2]))
                self.endPos = (self.startPos[0], self.startPos[1], math.radians(self.endPos[2]))
            self.isRadians = True
    """

    # start and end pos is in (x, y, theta), theta in radians
    # calculates start, end-coordinate, also arc center coordinates if applicable
    # call this AFTER units have been converted out of reeds-shepp format!!!!
    def setStartAndEndCoord(self, startPos, turningRadius = 1):
        startPosAngle = utilities.wrapAngle(startPos[2])
        if self.steering == 0:
            endPos = (self.gear*self.value*math.cos(startPosAngle) + startPos[0], self.gear*self.value*math.sin(startPosAngle) + startPos[1], startPosAngle)
        else:
            arcCenterX = startPos[0] - turningRadius * math.cos(startPosAngle - (math.pi / 2) * self.steering)
            arcCenterY = startPos[1] - turningRadius * math.sin(startPosAngle - (math.pi / 2) * self.steering)
            angleChange = self.value
            """
            if not self.isRadians:
                angleChange = math.radians(self.value)
            else:
                angleChange = self.value
            """

            endAngle = utilities.wrapAngle(startPosAngle + angleChange*self.steering*self.gear)
            endPos = (arcCenterX + turningRadius*math.cos(endAngle - (math.pi/2)*self.steering), arcCenterY + turningRadius*math.sin(endAngle - (math.pi/2)*self.steering), endAngle)
            self.arcCenter = (arcCenterX, arcCenterY)
        """
        if not self.isRadians:
            startPos = (startPos[0], startPos[1], math.degrees(startPos[2]))
            endPos = (endPos[0], endPos[1], math.degrees(endPos[2]))
        """
        self.startPos = startPos
        self.endPos = endPos

    #convert action to list of discrete instructions (velocity, steeringAngle) to be sent every instructionPeriod
    #ASSUME that output positive steering angle is left and negative steering angle is right, CHECK LATER
    #steeringAngle input is in radians
    #positive velocity is move forward, negative velocity is move backward
    def convertToDiscreteInstruction(self, instructionPeriod, velocity, steeringAngle, turningRadius):
        instructionList = []
        if self.steering == 0:
            actionTime = self.value/velocity
            instructionNumber = round(actionTime/instructionPeriod)
            for i in range(instructionNumber):
                instructionList.append((velocity*self.gear, 0))
        else:
            """
            if self.isRadians == False:
                actionTime = math.radians(self.value)*turningRadius/velocity
            else:
                actionTime = self.value * turningRadius / velocity
            """
            actionTime = self.value * turningRadius / velocity
            instructionNumber = round(actionTime / instructionPeriod)
            for i in range(instructionNumber):
                instructionList.append((velocity*self.gear, steeringAngle*self.steering))
        return instructionList



#Reeds-Shepp class
class Reeds_Shepp:
    # returns fastest path from source to target given turningRadius of the car
    # give coordinates of target and source in (x, y, theta), where theta is in degrees
    def __init__(self, target, source, turningRadius):
        self.target = target
        self.source = source
        self.turningRadius = turningRadius

    ###UTILITY FUNCTIONS
    # target converted to coordinates relative to source for Reeds-Shepp calculations
    def relativeCoordinateConversion(self, target, source):
        #\print("Original target coordinates: (" + str(target[0]) + ", " + str(target[1]) + ", " + str(target[2]) + ")")
        #print("Source coordinates: (" + str(source[0]) + ", " + str(source[1]) + ", " + str(source[2]) + ")")
        x1 = target[0] - source[0]
        y1 = target[1] - source[1]
        theta = target[2] - source[2]
        x = x1 * math.cos(target[2]) - y1 * math.sin(target[2])
        y = x1 * math.sin(target[2]) + y1 * math.cos(target[2])
        #print("After converting to relative coordinates, target is now: (" + str(x) + ", " + str(y) + ", " + str(
        #   theta) + ")")
        return (x, y, theta)

    # convert coordinates of target given turning radius to achieve unit turning radius for Reeds-Shepp
    def unitRadiusConversion(self, target, turningRadius):
        return (target[0] / turningRadius, target[1] / turningRadius, target[2])

    # transforms coordinates of target for calculations with flipped steering
    def flipSteeringTransform(self, target):
        return (-target[0], target[1], -target[2])

    # transforms coordinates of target for calculations with flipped gear
    def flipGearTransform(self, target):
        return (target[0], -target[1], -target[2])

    # transforms coordinates of target for calculations with flipped order
    def flipOrderTransform(self, target):
        x = target[0] * math.cos(target[2]) + target[1] * math.sin(target[2])
        y = target[0] * math.sin(target[2]) - target[1] * math.cos(target[2])
        return (x, y, target[2])
    ###

    ###REEDS-SHEPP EQUATIONS
    # Each formula assumes the car moves from (0,0,0) to (x,y,theta), with theta in radians
    # Unit turning radius of 1 and unit velocity of 1 is assumed
    # Cost of each path and set of actions is returned

    # if flipSteering == 0, steering direction follows function default, else if flipSteering == 1, flip steering direction
    # if flipGear == 0, gear direction follows function default, else if flipGear == 1, flip gear direction
    # if flipOrder == 0, order of actions will follow default sequence, if flipOrder == 1, order of actions will be reversed

    # Reeds-Shepp 8.1: left forward(t) straight forward(u) left forward(v)
    def LfSfLf(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] - math.sin(target[2])
        y = target[1] - 1 + math.cos(target[2])
        u, t = utilities.getPolar(x, y)
        v = utilities.wrapAngle(target[2] - t)
        # if utilities.isBetween(t, 0, math.pi/2) and utilities.isBetween(v, 0, math.pi/2): #if cannot find, relax range to [0,pi]

        actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
        actionSet.append(Action(0, (-1) ** flipGear, u))
        actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, v))
        if flipOrder != 0:
            actionSet.reverse()

        return abs(t) + abs(u) + abs(v), actionSet
        # return math.inf

    # Reeds-Shepp 8.2: left forward(t) straight forward(u) right forward(v)
    def LfSfRf(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] + math.sin(target[2])
        y = target[1] - 1 - math.cos(target[2])
        u1, t1 = utilities.getPolar(x, y)
        if u1 ** 2 >= 4:
            u = math.sqrt(u1 ** 2 - 4)
            t = utilities.wrapAngle(t1 + math.atan2(2, u))
            v = utilities.wrapAngle(t - target[2])
            # if utilities.isBetween(t, 0, math.pi/2) and utilities.isBetween(v, 0, math.pi/2): #if cannot find, relax range to [0,pi]

            actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
            actionSet.append(Action(0, (-1) ** flipGear, u))
            actionSet.append(Action(-(-1) ** flipSteering, (-1) ** flipGear, v))
            if flipOrder != 0:
                actionSet.reverse()

            return abs(actionSet[0].value) + abs(actionSet[1].value) + abs(actionSet[2].value), actionSet
        return math.inf, actionSet

    # Reeds-Shepp 8.3: left forward(t) right backward(u) left forward(v)
    # Referenced from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c as there are typos in original paper
    def LfRbLf(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] - math.sin(target[2])
        y = target[1] - 1 + math.cos(target[2])
        u1, t1 = utilities.getPolar(x, y)
        if u1 <= 4:
            a = math.acos(u1 / 4)
            t = utilities.wrapAngle(t1 + math.pi / 2 + a)
            u = utilities.wrapAngle(math.pi - 2 * a)
            v = utilities.wrapAngle(target[2] - t - u)
            # if utilities.isBetween(t, 0, math.pi) and utilities.isBetween(u, 0, math.pi) and utilities.isBetween(v, 0, math.pi):

            actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
            actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, u))
            actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, v))
            if flipOrder != 0:
                actionSet.reverse()

            return abs(actionSet[0].value) + abs(actionSet[1].value) + abs(actionSet[2].value), actionSet
        return math.inf, actionSet

    # Reeds-Shepp 8.4: left forward(t) right backward(u) left backward(v)
    # Referenced from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c as there are typos in original paper
    def LfRbLb(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] - math.sin(target[2])
        y = target[1] - 1 + math.cos(target[2])
        u1, t1 = utilities.getPolar(x, y)
        if u1 <= 4:
            a = math.acos(u1 / 4)
            t = utilities.wrapAngle(t1 + math.pi / 2 + a)
            u = utilities.wrapAngle(math.pi - 2 * a)
            v = utilities.wrapAngle(t + u - target[2])
            # if utilities.isBetween(t, 0, u) and utilities.isBetween(u, 0, math.pi/2) and utilities.isBetween(v, 0, u):

            actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
            actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, u))
            actionSet.append(Action((-1) ** flipSteering, -(-1) ** flipGear, v))
            if flipOrder != 0:
                actionSet.reverse()

            return abs(actionSet[0].value) + abs(actionSet[1].value) + abs(actionSet[2].value), actionSet
        return math.inf, actionSet

    # Reeds-Shepp 8.7: left forward(t) right forward(u) left backward(u) right backward(v)
    # Referenced from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c as there are typos in original paper
    def LfRfLbRb(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] + math.sin(target[2])
        y = target[1] - 1 - math.cos(target[2])
        u1, t1 = utilities.getPolar(x, y)
        if u1 <= 4:
            if u1 <= 2:
                a = math.acos((u1 + 2) / 4)
                t = utilities.wrapAngle(t1 + math.pi / 2 + a)
                u = utilities.wrapAngle(a)
                v = utilities.wrapAngle(target[2] - t + 2 * u)
            else:
                a = math.acos((u1 - 2) / 4)
                t = utilities.wrapAngle(t1 + math.pi / 2 - a)
                u = utilities.wrapAngle(math.pi - a)
                v = utilities.wrapAngle(target[2] - t + 2 * u)
            # if utilities.isBetween(t, 0, u) and utilities.isBetween(u, 0, math.pi/2) and utilities.isBetween(v, 0, u):

            actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
            actionSet.append(Action(-(-1) ** flipSteering, (-1) ** flipGear, u))
            actionSet.append(Action((-1) ** flipSteering, -(-1) ** flipGear, u))
            actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, v))
            if flipOrder != 0:
                actionSet.reverse()

            return abs(actionSet[0].value) + abs(2 * actionSet[1].value) + abs(actionSet[3].value), actionSet
        return math.inf, actionSet

    # Reeds-Shepp 8.8: left forward(t) right backward(u) left backward(u) right forward(v)
    # Referenced from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c as there are typos in original paper
    def LfRbLbRf(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] + math.sin(target[2])
        y = target[1] - 1 - math.cos(target[2])
        u1, t1 = utilities.getPolar(x, y)
        u2 = (20 - u1 ** 2) / 16
        if u1 <= 6 and utilities.isBetween(u2, 0, 1):
            u = math.acos(u2)
            a = math.asin(2 * math.sin(u) / u1)
            t = utilities.wrapAngle(t1 + math.pi / 2 + a)
            v = utilities.wrapAngle(t - target[2])
            # if utilities.isBetween(t, 0, u) and utilities.isBetween(u, 0, math.pi/2) and utilities.isBetween(v, 0, u):

            actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
            actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, u))
            actionSet.append(Action((-1) ** flipSteering, -(-1) ** flipGear, u))
            actionSet.append(Action(-(-1) ** flipSteering, (-1) ** flipGear, v))
            if flipOrder != 0:
                actionSet.reverse()

            return abs(actionSet[0].value) + abs(2 * actionSet[1].value) + abs(actionSet[3].value), actionSet
        return math.inf, actionSet

    # Reeds-Shepp 8.9: left forward(t) right backward(pi/2) straight backward(u) left backward(v)
    # Referenced from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c as there are typos in original paper
    def LfRbSbLb(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] - math.sin(target[2])
        y = target[1] - 1 + math.cos(target[2])
        u1, t1 = utilities.getPolar(-y, x)
        if u1 >= 2:
            u = math.sqrt(u1 ** 2 - 4) - 2
            if (u >= 0):
                a = math.atan2(2, u + 2)
                t = utilities.wrapAngle(t1 + math.pi / 2 + a)
                v = utilities.wrapAngle(t + math.pi / 2 - target[2])
                # if utilities.isBetween(t, 0, math.pi/2) and utilities.isBetween(v, 0, math.pi/2):

                actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
                actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, math.pi / 2))
                actionSet.append(Action(0, -(-1) ** flipGear, u))
                actionSet.append(Action((-1) ** flipSteering, -(-1) ** flipGear, v))
                if flipOrder != 0:
                    actionSet.reverse()

                return abs(actionSet[0].value) + abs(actionSet[1].value) + abs(actionSet[2].value) + abs(
                    actionSet[3].value), actionSet
        return math.inf, actionSet

    # Reeds-Shepp 8.10: left forward(t) right backward(pi/2) straight backward(u) right backward(v)
    # Referenced from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c as there are typos in original paper
    def LfRbSbRb(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] + math.sin(target[2])
        y = target[1] - 1 - math.cos(target[2])
        u1, t1 = utilities.getPolar(y, x)
        if u1 >= 2:
            t = utilities.wrapAngle(t1 + math.pi / 2)
            u = u1 - 2
            v = utilities.wrapAngle(target[2] - math.pi / 2 - t)
            # if utilities.isBetween(t, 0, math.pi/2) and utilities.isBetween(v, 0, math.pi/2):

            actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
            actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, math.pi / 2))
            actionSet.append(Action(0, -(-1) ** flipGear, u))
            actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, v))
            if flipOrder != 0:
                actionSet.reverse()

            return abs(actionSet[0].value) + abs(actionSet[1].value) + abs(actionSet[2].value) + abs(
                actionSet[3].value), actionSet
        return math.inf, actionSet

    # Reeds-Shepp 8.11: left forward(t) right backward(pi/2) straight backward(u) left backward(pi/2) right forward(v)
    def LfRbSbLbRf(self, target, flipSteering, flipGear, flipOrder):
        actionSet = []

        x = target[0] + math.sin(target[2])
        y = target[1] - 1 - math.cos(target[2])
        u1, t1 = utilities.getPolar(x, y)
        if u1 >= 4:
            u = math.sqrt(u1 ** 2 - 4) - 4
            if (u >= 0):
                a = math.atan2(2, u + 4)
                t = utilities.wrapAngle(t1 + math.pi / 2 + a)
                v = utilities.wrapAngle(t - target[2])
                # if utilities.isBetween(t, 0, math.pi/2) and utilities.isBetween(v, 0, math.pi/2):

                actionSet.append(Action((-1) ** flipSteering, (-1) ** flipGear, t))
                actionSet.append(Action(-(-1) ** flipSteering, -(-1) ** flipGear, math.pi / 2))
                actionSet.append(Action(0, -(-1) ** flipGear, u))
                actionSet.append(Action((-1) ** flipSteering, -(-1) ** flipGear, math.pi / 2))
                actionSet.append(Action(-(-1) ** flipSteering, (-1) ** flipGear, v))
                if flipOrder != 0:
                    actionSet.reverse()

                return abs(actionSet[0].value) + math.pi + abs(actionSet[2].value) + abs(actionSet[4].value), actionSet
        return math.inf, actionSet
    ###


    #METHOD TO RUN THE PATHFINDING ALGO
    def run(self):
        # convert target coordinates to be relative to source

        target = self.relativeCoordinateConversion(self.target, self.source)

        # convert units to achieve unit radius
        target = self.unitRadiusConversion(target, self.turningRadius)
        # print("After conversion using turningRadius, target is now: (" + str(target[0]) + ", " + str(target[1]) + ", " + str(target[2]) + ")")

        reedsSheppFunctions = [self.LfSfLf, self.LfSfRf, self.LfRbLf, self.LfRbLb, self.LfRfLbRb, self.LfRbLbRf, self.LfRbSbLb, self.LfRbSbRb, self.LfRbSbLbRf]
        flip = [0, 1]
        successCost = math.inf
        successActionSet = None

        for fn in reedsSheppFunctions:
            for flipSteering in flip:
                for flipGear in flip:
                    for flipOrder in flip:
                        if flipSteering == 1:
                            target = self.flipSteeringTransform(target)
                        if flipGear == 1:
                            target = self.flipGearTransform(target)
                        if flipOrder == 1:
                            target = self.flipOrderTransform(target)
                        cost, actionSet = fn(target, flipSteering, flipGear, flipOrder)
                        if cost < successCost:
                            successCost = cost
                            successActionSet = actionSet

        # remove actions that have no value
        for action in successActionSet:
            if action.value == 0:
                successActionSet.remove(action)

        # convert successActionSet units back to original units (reverse the radius transform)
        # turning actions are in radians
        for action in successActionSet:
            action.convertValueOriginalScale(self.turningRadius)

        return successCost, successActionSet

