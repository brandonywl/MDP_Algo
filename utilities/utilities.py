import math

def cm_to_pixel(cm, map_shape=800, physical_shape=200):
    """
    Converts cm to pixel. Expects a square shaped map
    :param cm: Input cm
    :param map_shape: Shape of the map (1-axis) in pixels. Preferred to be a multiple of 200 for round numbers.
    :param physical_shape: Shape of the map (1-axis) in real life in cm. Expected 200
    :return: Number of pixels to represent the cm. Will round off floats.
    """

    return round((map_shape / physical_shape) * cm)

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

if __name__ == "__main__":
    print(cm_to_pixel(50))
