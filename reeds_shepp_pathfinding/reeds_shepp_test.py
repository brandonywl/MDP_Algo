from reeds_shepp_pathfinding.reeds_shepp import Reeds_Shepp
from utilities import drawing
import math

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