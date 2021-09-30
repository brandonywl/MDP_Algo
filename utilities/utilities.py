import math
import reeds_shepp_pathfinding.reeds_shepp as reeds_shepp

def cm_to_pixel(cm, map_shape=800, physical_shape=200):
    """
    Converts cm to pixel. Expects a square shaped map
    :param cm: Input cm
    :param map_shape: Shape of the map (1-axis) in pixels. Preferred to be a multiple of 200 for round numbers.
    :param physical_shape: Shape of the map (1-axis) in real life in cm. Expected 200
    :return: Number of pixels to represent the cm. Will round off floats.
    """

    return round((map_shape / physical_shape) * cm)

def pixel_to_cm(pixel, map_shape=800, physical_shape=200):
    return (physical_shape / map_shape) * pixel

# checks if value is between lower and upper (inclusive of)
def isBetween(value, lower, upper):
    if value >= lower and value <= upper:
        return True
    else:
        return False

# converts angle to be between 0 and 2pi
def wrapAngle(value):
    while value > 2 * math.pi:
        value -= 2 * math.pi
    while value < 0:
        value += 2 * math.pi
    return value

# converts angle to be between -180 and 180
def wrapAnglePI(value):
    while value > 180:
        value -= 360
    while value < -180:
        value += 360
    return value

# returns polar coordinates (r, theta) of the point (x, y), theta in radians
def getPolar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    theta = math.atan2(y, x)
    return r, theta

#convert pycharm coordinates to fourth quadrant of normal coordinate system
#retain x-coordinate value, divide y-coordinate by -1
#pycharmCoord is in form (x, y, theta), theta in radians
def convertCoordFromPyCharm(pycharmCoord):
    newCoord = (pycharmCoord[0], -pycharmCoord[1], pycharmCoord[2])
    return newCoord

#convert discrete instructions to hardware
#instructionSet in the form [(velocity, steeringAngle),...], assuming steeringAngle is between 180 and -180
def discreteInstructionToHardware(instructionSet, vehicle_length, interval = 1):
    #W is straight
    #A is left
    #D is right
    #HWInstructions in the form [('W/A/D', angle/distance),...], angle in degree, distance in irl values
    turningRadius = pixel_to_cm(float(vehicle_length)) / math.tan(math.radians(40))
    HWInstructions = []
    totalValue = 0
    currentSteeringAngle = instructionSet[0][1]
    for i in range(len(instructionSet)):
        instruction = instructionSet[i]
        nextSteeringAngle = instruction[1]
        if nextSteeringAngle != currentSteeringAngle:
            if currentSteeringAngle == 0:
                action = 'W'
            elif currentSteeringAngle > 0:
                action = 'A'
            else:
                action = 'D'
            HWInstructions.append((action, totalValue))
            totalValue = 0
        currentSteeringAngle = nextSteeringAngle
        #straight motion
        if currentSteeringAngle == 0:
            #update total distance
            totalValue += pixel_to_cm(instruction[0] * interval)
            #totalValue += 20
        #turning motion
        else:
            #update total angle
            #totalValue += math.degrees(pixel_to_cm(instruction[0] * interval)/pixel_to_cm(turningRadius))
            #it wants to turn instruction[1] degrees and travel instruction[0] distance,
            # but instruction[1] degrees = instruction[0]/turningRadius
            totalValue += math.degrees(pixel_to_cm(instruction[0] * interval) / turningRadius)
            # totalValue += abs(instruction[1] * interval)
        if i == (len(instructionSet) - 1): #current element is last element
            if currentSteeringAngle == 0:
                action = 'W'
            elif currentSteeringAngle > 0:
                action = 'A'
            else:
                action = 'D'
            HWInstructions.append((action, totalValue))

    return HWInstructions

#convert discrete instructions to hardware
#instructionSet in the form [(velocity, steeringAngle),...], assuming steeringAngle is between 180 and -180
def discreteInstructionToHardware2(instructionSet, turningRadius, interval = 1):
    #W is straight
    #A is left
    #D is right
    #HWInstructions in the form [('W/A/D', angle/distance),...], angle in degree, distance in irl values
    turningRadius = 35
    HWInstructions = []
    for i in range(len(instructionSet)):
        instruction = instructionSet[i]
        steeringAngle = instruction[1]
        value = instruction[0]
        if steeringAngle == 0:
            HWInstructions.append(('W', value))
        else:
            source = (0,0,0)
            target = (value*math.cos(0),value*math.sin(0),steeringAngle)
            RS = reeds_shepp.Reeds_Shepp(target, source, turningRadius)
            output = RS.run()
            actionSet = output[1]
            for action in actionSet:
                if action.steering == 0:
                    HWInstructions.append(('W', action.value*action.gear))
                elif action.steering == 1:
                    HWInstructions.append(('A', math.degrees(action.value * action.gear)))
                elif action.steering == -1:
                    HWInstructions.append(('D', math.degrees(action.value * action.gear)))
    return HWInstructions

if __name__ == "__main__":
    print(cm_to_pixel(50))
