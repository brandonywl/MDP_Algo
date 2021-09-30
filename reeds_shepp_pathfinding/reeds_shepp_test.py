import time
import math
import heapq
import pygame

from reeds_shepp_pathfinding.reeds_shepp import Reeds_Shepp, Action
from reeds_shepp_pathfinding.simplePathfinding import simplePathfinding
from reeds_shepp_pathfinding import reeds_shepp_collision_detection
from utilities import drawing, utilities
from Env import Env
from map_tiles.block import Block

block_1 = (0, 10, 0)
block_2 = (6, 15, 0)
block_3 = (10, 8, 180)
block_4 = (6, 5, 270)
blocks_pos = (block_1, block_2, block_3, block_4)

blocks = Block.create_blocks(blocks_pos, (drawing.CUBE_WIDTH, drawing.CUBE_LENGTH))
block_pos = [x.get_pos()[1:] for x in blocks]
block_target_pos = [tuple(Block.get_target_point(block).as_list(False)) for block in blocks]

# Initialize environment
env = Env(num_blocks=5, display=True)
env.blocks = blocks
env.update()

# initialise info about robot
# WHEEL_LENGTH is 20
fwheelbwheelDist = drawing.CAR_LENGTH
# steeringAngle in degrees
steeringAngle = math.radians(env.STEERING_ANGLE)
turningRadius = fwheelbwheelDist / math.tan(steeringAngle)
# turningRadius = 25
carSpd = env.MOVE_SPEED_PIXEL
# send an instruction every s, dummy value here
instructionPeriod = 1

def reedsSheppTestStraightMotion(robot_start_pos):
    print("Testing forward movement 100 pixels")
    target_node_pos = (robot_start_pos[0], robot_start_pos[1] + 100, robot_start_pos[2])
    reedsShepp = Reeds_Shepp(target_node_pos, robot_start_pos, turningRadius)
    cost, actionSet = reedsShepp.run()
    print("Starting point: ", robot_start_pos)
    print("Target : ", target_node_pos)
    print("Cost:", cost)
    print("ActionSet:")
    for a in actionSet:
        print("L/R/S: " + str(a.steering) + ", F/B: " + str(a.gear) + ", Value: " + str(a.value))
        a.setStartAndEndCoord(robot_start_pos, turningRadius)
        print("Start position: ", a.startPos)
        print("End position: ", a.endPos)
        drawing.draw_path_action(env.window, a, turningRadius)
    print()

def reedsSheppTestTurningMotion(robot_start_pos):
    print("Testing turning movement reverse on the spot")
    target_node_pos = (robot_start_pos[0], robot_start_pos[1], robot_start_pos[2] + math.pi)
    reedsShepp = Reeds_Shepp(target_node_pos, robot_start_pos, turningRadius)
    cost, actionSet = reedsShepp.run()
    print("Starting point: ", robot_start_pos)
    print("Target : ", target_node_pos)
    print("Cost:", cost)
    print("ActionSet:")
    startPos = robot_start_pos
    for a in actionSet:
        print("L/R/S: " + str(a.steering) + ", F/B: " + str(a.gear) + ", Value: " + str(a.value))
        a.setStartAndEndCoord(startPos, turningRadius)
        print("Start position: ", a.startPos)
        print("End position: ", a.endPos)
        drawing.draw_path_action(env.window, a, turningRadius)
        startPos = a.endPos
    print()

#outputs order of goals to visit using reeds shepp
#returns sortedFoundPaths, which has the format [(cost, [(target_node_pos), [actionSet]]),..]
def findGoalOrderReedsShepps(robot_start_pos, goals):
    # foundPathsHeap = { (cost, (target_node_pos), [actionSet]), ...}
    foundPathsHeap = []
    # finalPath = [ ((target_node_pos), [actionSet], cost), ..]
    finalPath = []

    print("All paths sorted: \n")
    count = 1
    while goals:
        for target_node_pos in goals:
            reedsShepp = Reeds_Shepp(target_node_pos, robot_start_pos, turningRadius)
            cost, actionSet = reedsShepp.run()
            heapq.heappush(foundPathsHeap, (cost, target_node_pos, actionSet))

        sortedFoundPath = heapq.heappop(foundPathsHeap)

        # add the path to the nearest target
        finalPath.append([sortedFoundPath[1], sortedFoundPath[2], sortedFoundPath[0]])

        print("Starting point ", count, ": ", robot_start_pos)
        print("Target ", count, ": ", finalPath[-1][0])
        print("Cost:", finalPath[-1][2])
        print("ActionSet:")
        startPos = robot_start_pos
        for a in finalPath[-1][1]:
            print("L/R/S: " + str(a.steering) + ", F/B: " + str(a.gear) + ", Value: " + str(a.value))
            a.setStartAndEndCoord(startPos, turningRadius)
            drawing.draw_path_action(env.window, a, turningRadius)
            pygame.display.update()
            startPos = a.endPos
        print()
        count += 1

        robot_start_pos = finalPath[-1][0]
        if startPos != robot_start_pos:
            print("Calculated end position does not equal next start position!")
            print("Calculated end pos: ", startPos)
            print("Actual end pos: ", robot_start_pos)
            print()
        goals.remove(robot_start_pos)
        foundPathsHeap.clear()
    return finalPath

def findMidPoint(start, end, obstacles):
    simplePath = simplePathfinding(start, end, obstacles)
    simplePath.run()
    return simplePath.findPathMidPoint()

def checkActionSetObstacleCollision(path, startPos, obstacles):
    newPath = []
    for actionSet in path:
        Action.setStartAndEndCoordActionSet(actionSet, startPos, turningRadius)
        collision = reeds_shepp_collision_detection.collisionDetectionActionSet(startPos, actionSet, obstacles,
                                                                                turningRadius)
        endPos = actionSet[-1].endPos
        if collision:
            midPos = findMidPoint(startPos, endPos, obstacles)
            reedsShepp1 = Reeds_Shepp(startPos, midPos, turningRadius)
            reedsShepp2 = Reeds_Shepp(midPos, endPos, turningRadius)
            actionSet1 = reedsShepp1.run()[1]
            actionSet2 = reedsShepp2.run()[1]
            path1 = checkActionSetObstacleCollision(actionSet1,startPos,obstacles)
            path2 = checkActionSetObstacleCollision(actionSet2,midPos,obstacles)
            for a in path1:
                newPath.append(a)
            for b in path2:
                newPath.append(b)
            return newPath
        else:
            newPath.append(actionSet)
        startPos = endPos
    return newPath

def printAllPaths(allPaths):
    obstacleCount = 1
    actionSetCount = 1
    for path in allPaths:
        print("Obstacle " + str(obstacleCount) + ":")
        for actionSet in path:
            print("ActionSet " + str(actionSetCount) + ":")
            for action in actionSet:
                print("L/R/S: " + str(action.steering) + ", F/B: " + str(action.gear) + ", Value: " + str(action.value))
            actionSetCount += 1
        actionSetCount = 1
        obstacleCount += 1
    print()

if __name__ == "__main__":

    # Get (robot_start_x, robot_start_y, robot_start_theta (degrees))
    robot_start_pos_degrees = tuple(env.robot.get_point().as_list(False))
    # convert degrees to radians and move origin to bottom left
    robot_start_pos = utilities.convertCoordFromPyCharm((robot_start_pos_degrees[0], robot_start_pos_degrees[1], math.radians(robot_start_pos_degrees[2])))
    print("Start node:", robot_start_pos)
    #reedsSheppTestStraightMotion(robot_start_pos)
    #reedsSheppTestTurningMotion(robot_start_pos)


    # goals = [(goal1), (goal2), ...]
    goals = []   

    for block in env.blocks:
        # Get (target_x, target_y, target_theta (degrees))
        block_pos = tuple(Block.get_target_point(block).as_list(False))
        # convert degrees to radians and move origin to bottom left
        goals.append(utilities.convertCoordFromPyCharm((block_pos[0], block_pos[1], math.radians(block_pos[2]))))

    print("All goals: ")
    print(goals)

    # prelimPath = [ ((target_node_pos), [actionSet], cost), ..]
    prelimPath = findGoalOrderReedsShepps(robot_start_pos, goals)

    #each path contains a list of actionSets, which are lists of actions themselves
    path1 = []
    path2 = []
    path3 = []
    path4 = []
    path5 = []

    path1.append(prelimPath[0][1])
    path2.append(prelimPath[1][1])
    path3.append(prelimPath[2][1])
    path4.append(prelimPath[3][1])
    #path5.append(prelimPath[4][1])

    allPaths = [path1, path2, path3, path4, path5]
    obstacles = env.blocks
    startPos = robot_start_pos

    print("Before collision checking: ")
    printAllPaths(allPaths)

    # update the start and end coordinates of every action
    for path in allPaths:
        actionSet = path[0]
        endPos = actionSet[-1].endPos
        path = checkActionSetObstacleCollision(path, startPos, obstacles)
        startPos = endPos

    print("After collision checking: ")
    printAllPaths(allPaths)



    # drawing.draw_path_action(env.window, action, turningRadius)
    pygame.display.update()

    #pygame

    left_press = right_press = up_press = down_press = 0
    while not env.done:
        if True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("env.done set: Quit")
                    env.done = True
                    break
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT:
                        left_press = 1
                    elif event.key == pygame.K_RIGHT:
                        right_press = 1
                    elif event.key == pygame.K_UP:
                        up_press = 1
                    elif event.key == pygame.K_DOWN:
                        down_press = 1
                    elif event.key == pygame.K_ESCAPE:
                        env.reset_robot()
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_RIGHT:
                        right_press = 0
                    elif event.key == pygame.K_LEFT:
                        left_press = 0
                    if event.key == pygame.K_UP:
                        up_press = 0
                    elif event.key == pygame.K_DOWN:
                        down_press = 0
            # If steering = 0, either both or none pressed. If steering = 1, left press, steering = -1, right press
            steering_direction = left_press - right_press
            movement_direction = up_press - down_press

            if movement_direction == 0:
                curr_action = 0

            elif steering_direction == -1:
                curr_action = 1 if movement_direction == 1 else 4
            elif steering_direction == 1:
                curr_action = 3 if movement_direction == 1 else 6
            else:
                curr_action = 2 if movement_direction == 1 else 5





    






"""
a_star = Hybrid_AStar(robot_start_pos, target_node_pos, env.blocks)
output = a_star.run()
if env.planned_path is None:
    env.planned_path = output[0]
else:
    env.planned_path.extend(output[0])

robot_start_pos = output[0][-1][1]
"""

"""
# Generate blocks for the map
#blocks = Block.generate_blocks((CUBE_WIDTH, CUBE_LENGTH), 5)
#robot = Robot(np.radians(90), CAR_WIDTH/2 + WINDOW_OFFSET_WIDTH, (WINDOW_HEIGHT - CAR_LENGTH/4 + WINDOW_OFFSET_HEIGHT), CAR_WIDTH, CAR_LENGTH)
#block_pos = [x.get_pos()[1:] for x in blocks]


#goals = [(0, 25, 0), (0, 5, 90), (25, 100, -90), (20, 0, 0), (0, 0, 180), (-25, -75, 180)]
goals = [(25, 0, 0)]
start = (0, 0, 0)
#WHEEL_LENGTH is 20
fwheelbwheelDist = drawing.WHEEL_LENGTH
#steeringAngle in degrees
steeringAngle = 25
#turningRadius = fwheelbwheelDist/math.tan(math.radians(steeringAngle))
turningRadius = 25
carSpd = 10
#send an instruction every s, dummy value here
instructionPeriod = 1


for i in goals:
    reedsShepp = Reeds_Shepp(i, start, turningRadius)
    cost, actionSet = reedsShepp.run()
    print("Cost of (" + str(i[0]) + ", " + str(i[1]) + ", " + str(i[2]) + ") is: " + str(cost))
    print("Action set of (" + str(i[0]) + ", " + str(i[1]) + ", " + str(i[2]) + ") is: ")
    instructionSetPerPath = []
    for a in actionSet:
        print("L/R/S: " + str(a.steering) + ", F/B: " + str(a.gear) + ", Value: " + str(a.value))
        instructionSetPerAction = a.convertToDiscreteInstruction(instructionPeriod, carSpd, steeringAngle, turningRadius)
        for ins in instructionSetPerAction:
            instructionSetPerPath.append(ins)
    print("Instruction set of (" + str(i[0]) + ", " + str(i[1]) + ", " + str(i[2]) + ") is: ")
    for ins in instructionSetPerPath:
        print("velocity: " + str(ins[0]) + ", steering angle: " + str(ins[1]))
    print()
"""
