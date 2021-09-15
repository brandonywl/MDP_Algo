import math

from movement_models import collision_detection
from reeds_shepp_pathfinding import reeds_shepp
from utilities import drawing
from map_tiles.point import Point

#detects collision between Action and Block
#assumes start is (x, y, theta), theta in degrees
def collisionDetectionStraightAction(start, action, obstacles):
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

    #straight motion
    if action.steering == 0:
        startX = start[0]
        startY = start[1]
        length = action.value
        if action.gear == 1: #forward motion
            angle = math.radians(start[2])
        else: #backward motion
            angle = math.radians(start[2] - 180)
        for obstacle in obstaclesCoord:
            nearestDist = math.sqrt((obstacle[0] - startX)**2 + (obstacle[1] - startY)**2)
            if nearestDist <= maxNearestDist1:
                return True
            elif nearestDist <= maxNearestDist2:
                #check if car and obstacle are truly collided
                deltaX = length*math.cos(angle)
                deltaY = length*math.sin(angle)
                gradient = deltaY/deltaX
                carX = (startY - startX - deltaX*obstacle[0] - deltaY*obstacle[1])/(deltaX + deltaY*gradient)
                carY = gradient*carX - startX + startY
                return collision_detection.has_collision(Point(carX, carY, start[2]), [obstacle,])
    #arc motion
    #else:

    return False

