import numpy as np

from map_tiles.point import Point


def move(point, turn, forward_velocity, robot_length, robot_max_steering_angle, frequency=1):
    """
    Moves a single timestep given turn and forward. How long a timestep is can be controlled with frequency.
    :param point: Starting point. Requires radians
    :param turn: Steering angle of the rear-wheeled vehicle. Requires radians
    :param forward_velocity: Velocity of the car in pixel/instance or pixel/s.
    :param frequency: Controls delta time factor to convert to distance. If forward_velocity is provided in cm/s
    and converted to pixels/s, frequency will be 1/T where T is how long we want to stay in each command for the
    robot to move.
    :return: x,y, theta of the next state
    """
    theta = point.theta  # initial orientation
    alpha = turn  # steering angle
    dist = forward_velocity / frequency  # distance to be moved
    length = robot_length  # length of the robot
    if abs(alpha) > robot_max_steering_angle:
        raise ValueError('Exceeding max steering angle')

    # Uncomment to only allow single-direction movement
    # if dist < 0.0:
    #     raise ValueError('Moving backwards is not valid')

    # in local coordinates of robot
    beta = (dist / length) * np.tan(alpha)  # turning angle
    # print degrees(beta)
    _x = _y = _theta = 0.0
    if beta > 0.001 or beta < -0.001:
        radius = dist / beta  # turning radius
        cx = point.x - np.sin(theta) * radius  # center of the circle
        cy = point.y - np.cos(theta) * radius  # center of the circle

        # Uncomment to see the center of the circle the robot is going about
        # pygame.draw.circle(screen, red, (int(cx), int(cy)), 5)
        # pygame.display.update()

        # in global coordinates of robot
        _x = cx + np.sin(theta + beta) * radius
        _y = cy + np.cos(theta + beta) * radius
        _theta = (theta + beta) % (2 * np.pi)

    else:  # straight motion
        _x = point.x + dist * np.cos(theta)
        _y = point.y - dist * np.sin(theta)
        _theta = (theta + beta) % (2 * np.pi)

    final_point = Point(_x, _y, _theta)
    # self.steering_angle = alpha

    # self.x %= world_size
    # self.y %= world_size
    return final_point

def backaxel_move(point, velocity, steering_angle, vehicle_length, frequency=1):
    """

    :param point: Starting point. Theta in radians
    :param velocity: Velocity in pixel/s
    :param steering_angle: Steering angle in radians
    :param vehicle_length: Vehicle length in pixels
    :return: Next point
    """
    velocity /= frequency
    new_x = point.x + np.cos(point.theta) * velocity
    new_y = point.y - np.sin(point.theta) * velocity
    new_theta = point.theta + (velocity * np.tan(steering_angle) / (float(vehicle_length)))
    return Point(new_x, new_y, new_theta)
