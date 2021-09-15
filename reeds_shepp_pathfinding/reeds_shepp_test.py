import time
import math
import heapq
import pygame

from reeds_shepp_pathfinding.reeds_shepp import Reeds_Shepp
from utilities import drawing
from Env import Env
from map_tiles.block import Block

# Initialize environment
env = Env(num_blocks=5, display=True)
# initialise info about robot
# WHEEL_LENGTH is 20
fwheelbwheelDist = drawing.CAR_LENGTH
# steeringAngle in degrees
steeringAngle = env.STEERING_ANGLE
turningRadius = fwheelbwheelDist/math.tan(math.radians(steeringAngle))
# turningRadius = 25
carSpd = env.MOVE_SPEED_PIXEL
# send an instruction every s, dummy value here
instructionPeriod = 1

def checkActionSetObstacleCollision():
    return True

#outputs order of goals to visit using reeds shepp
#returns sortedFoundPaths, which has the format [(cost, [(target_node_pos), [actionSet]]),..]
def findGoalOrderReedsShepps(robot_start_pos, goals):
    # foundPaths = { cost: [(target_node_pos), [actionSet]], ...}
    # foundPaths = {}
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
            #foundPaths[cost] = [target_node_pos, actionSet]

        # sortedFoundPaths has the format (cost, (target_node_pos), [actionSet])
        sortedFoundPath = heapq.heappop(foundPathsHeap)
        #sortedFoundPaths = sorted(foundPaths.items())
        # add the path to the nearest target
        finalPath.append([sortedFoundPath[1], sortedFoundPath[2], sortedFoundPath[0]])

        print("Starting point ", count, ": ", robot_start_pos)
        print("Target ", count, ": ", finalPath[-1][0])
        print("Cost:", finalPath[-1][2])
        print("ActionSet:")
        for a in finalPath[-1][1]:
            print("L/R/S: " + str(a.steering) + ", F/B: " + str(a.gear) + ", Value: " + str(a.value))
        print()
        count += 1

        robot_start_pos = finalPath[-1][0]
        goals.remove(robot_start_pos)
        foundPathsHeap.clear()


if __name__ == "__main__":

    # Get (robot_start_x, robot_start_y, robot_start_theta (degrees))
    robot_start_pos = tuple(env.robot.get_point().as_list(False))
    print("Start node:", robot_start_pos)

    # goals = [(goal1), (goal2), ...]
    goals = []   

    for block in env.blocks:
        # Get (target_x, target_y, target_theta (degrees))
        goals.append(tuple(Block.get_target_point(block).as_list(False)))

    print("All goals: ")
    print(goals)

    sortedFoundPaths = findGoalOrderReedsShepps(robot_start_pos, goals)

    






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
