import math

from movement_models import collision_detection
from reeds_shepp_pathfinding import reeds_shepp
from utilities import drawing
from map_tiles.point import Point

#where range is (lower bounds, upper bounds)
#returns 0 if angle in range, 1 if it is larger than upper bound, -1 if it is smaller than lower bound
def angleInRange(range, angle):
    halfRange = (range[1] - range[0])/2
    rangeMidPoint = range[1] - halfRange
    #ensure both values are between 0 and 2pi
    rangeMidPoint = reeds_shepp.wrapAngle(rangeMidPoint)
    angle = reeds_shepp.wrapAngle(angle)
    if abs(angle - rangeMidPoint) <= halfRange:
        return 0
    elif angle > rangeMidPoint:
        return 1
    else:
        return -1

#detects collision between Action and Block
#assumes start of action is (x, y, theta), theta in degrees
def collisionDetectionStraightAction(start, action, obstacles, turningRadius = 0):
    #assume that obstacles are square
    obstacleWidth = obstacles[0].cell_width
    obstaclesCoord = []
    #get list of center points of blocks
    for i in obstacles:
        obstaclesCoord.append((i.x + i.obstacleWidth/2, i.y + i.obstacleWidth/2))
    
    #if distance between obstacle center and car position < maxNearestDist1, confirm collision
    maxNearestDist1 = drawing.CAR_LENGTH/4 + obstacleWidth/2
    #if distance between obstacle center and car position < maxNearestDist2, do in depth collision checking
    maxNearestDist2 = math.sqrt((drawing.CAR_LENGTH*(3/4))**2 + (drawing.CAR_WIDTH/2)**2) + math.sqrt((obstacleWidth**2)/2)

    startX = start[0]
    startY = start[1]
    theta = math.radians(start[2])

    #straight motion
    if action.steering == 0:
        length = action.value
        if action.gear == -1: #backward motion
            theta = theta - math.pi
        for obstacle in obstaclesCoord:
            nearestDist = math.sqrt((obstacle[0] - startX)**2 + (obstacle[1] - startY)**2)
            if nearestDist <= maxNearestDist1:
                return True
            elif nearestDist <= maxNearestDist2:
                #check if car and obstacle are truly collided
                deltaX = length*math.cos(theta)
                deltaY = length*math.sin(theta)
                gradient = deltaY/deltaX
                carX = (deltaX*obstacle[0] + deltaY*obstacle[1] - deltaY*startY + deltaY*gradient*startX)/(deltaX + deltaY*gradient)
                carY = gradient*(carX - startX) + startY
                return collision_detection.has_collision(Point(carX, carY, start[2]), [obstacle,])
    #arc motion
    else:
        startArcAngle = theta - (math.pi/2)*action.steering
        arcCenterX = startX - turningRadius*math.cos(startArcAngle)
        arcCenterY = startY - turningRadius*math.sin(startArcAngle)
        #anti-clockwise movement (left forward or right backward)
        if action.steering == action.gear:
            arcAngleRange = (startArcAngle, startArcAngle + action.value)
        #clockwise movement (left backward or right forward)
        else:
            arcAngleRange = (startArcAngle - action.value, startArcAngle)
        for obstacle in obstaclesCoord:
            obstacleDistToCenter = math.sqrt((obstacle[0] - arcCenterX)**2 + (obstacle[1] - arcCenterY)**2)
            obstacleAngle = math.atan2((obstacle[1] - arcCenterY)/(obstacle[0] - arcCenterX))
            inRange = angleInRange(arcAngleRange, obstacleAngle)
            if inRange == 0:
                obstacleDistToNearestPoint = math.abs(obstacleDistToCenter - turningRadius)
                if obstacleDistToNearestPoint < maxNearestDist1:
                    return True
                elif obstacleDistToNearestPoint < maxNearestDist2:
                    #check if car and obstacle are truly collided
                    carX = startX + (turningRadius/obstacleDistToCenter)*(obstacle[0] - arcCenterX)
                    carY = startY + (turningRadius/obstacleDistToCenter)*(obstacle[1] - arcCenterY)
                    return collision_detection.has_collision(Point(carX, carY, obstacleAngle + (math.pi/2)*action.steering), [obstacle,])
            else:
                carTheta = arcAngleRange[-inRange]
                carX = turningRadius*math.cos(carTheta)
                carY = turningRadius*math.sin(carTheta)
                return collision_detection.has_collision(Point(carX, carY, carTheta + (math.pi/2)*action.steering), [obstacle,])          
    return False

