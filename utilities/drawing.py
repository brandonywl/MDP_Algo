import numpy as np
import pygame

RED = (200, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 155, 0)
YELLOW = (200, 200, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (67, 70, 75)
CYAN = (0, 255, 255)

WINDOW_WIDTH, WINDOW_HEIGHT = 600, 600
WINDOW_OFFSET_WIDTH, WINDOW_OFFSET_HEIGHT = 100, 100


CUBE_LENGTH = WINDOW_HEIGHT // 20
CUBE_WIDTH = WINDOW_WIDTH // 20
CUBE_COLOR = CYAN
CUBE_COLORS = [RED, BLUE, GREEN, YELLOW]

CAR_LENGTH = WINDOW_HEIGHT / 10
CAR_WIDTH = WINDOW_WIDTH / 10
CAR_COLOR = GREY

WHEEL_LENGTH = 20
WHEEL_WIDTH = 6


def compute_corners(center, corners, rotation_angle):
    return [rotate_point(center[0], center[1], x[0], x[1], rotation_angle) for x in corners]


def rotate_point(center_x, center_y, point_x, point_y, rotation_angle):
    length = np.sqrt(np.square(point_x - center_x) + np.square(point_y - center_y))
    angle = np.arctan2(center_y - point_y, point_x - center_x)
    angle += rotation_angle

    new_x = center_x + length * np.cos(angle)
    new_y = center_y - length * np.sin(angle)
    return new_x, new_y


def get_corners(center_x, center_y, arg1, arg2, arg3=None, arg4=None):
    arg3 = arg1 if arg3 is None else arg3
    arg4 = arg2 if arg4 is None else arg4

    p1 = [center_x - arg1, center_y - arg2]
    p2 = [center_x + arg3, center_y - arg2]
    p3 = [center_x + arg3, center_y + arg4]
    p4 = [center_x - arg1, center_y + arg4]

    return [p1, p2, p3, p4]


def get_block_corners(block):
    return get_corners(block.x, block.y, 0, 0, block.cell_height, block.cell_width)


def get_wheel_corners(wheel_c_x, wheel_c_y, wheel_length, wheel_width):
    wheel_x = wheel_length / 2
    wheel_y = wheel_width / 2

    results = get_corners(wheel_c_x, wheel_c_y, wheel_x, wheel_y)

    return results


def get_wheels_centers(car_x, car_y, car_length, car_width, orientation):
    wheels = get_corners(car_x, car_y, 0, car_width/3, car_length / 2)
    adjusted_wheels = [rotate_point(car_x, car_y, wheel[0], wheel[1], orientation) for wheel in wheels]
    return adjusted_wheels


# Helper function to draw rectangle with rotation
def draw_rect(window, center, corners, rotation_angle, color):

    rotated_corners = compute_corners(center, corners, rotation_angle)

    # draw rectangular polygon --> car body
    rect = pygame.draw.polygon(window, color,
                               (rotated_corners[0], rotated_corners[1], rotated_corners[2], rotated_corners[3]))
    return rect, rotated_corners


def draw_rect_no_rotate(window, center, rotated_corners, rotation_angle, color):
    rect = pygame.draw.polygon(window, color,
                               (rotated_corners[0], rotated_corners[1], rotated_corners[2], rotated_corners[3]))
    return rect, rotated_corners


def get_robot_corners(car_x, car_y, rotation_angle):
    corners = get_corners(car_x, car_y, CAR_LENGTH/4, CAR_WIDTH/2, 0.75*CAR_LENGTH, CAR_WIDTH/2)
    rotated_corners = compute_corners([car_x, car_y], corners, rotation_angle)
    return rotated_corners


def draw_robot(robot, window):
    car_x = robot.x
    car_y = robot.y
    orientation = robot.theta
    steering_angle = robot.steering_angle

    # car body
    rotated_corners = get_robot_corners(car_x, car_y, orientation)
    car, rotated_corners = draw_rect_no_rotate(window, [car_x, car_y], rotated_corners, orientation, CAR_COLOR)

    # wheels
    wheels = get_wheels_centers(car_x, car_y, CAR_LENGTH, CAR_WIDTH, orientation)
    for idx, wheel_center in enumerate(wheels):
        wheel_corners = get_wheel_corners(wheel_center[0], wheel_center[1], WHEEL_LENGTH, WHEEL_WIDTH)
        wheel_orientation = orientation if idx == 0 or idx == 3 else orientation + steering_angle
        draw_rect(window, wheel_center, wheel_corners, wheel_orientation, BLACK)

    # draw axle
    pygame.draw.line(window, BLACK, wheels[0], wheels[3], 1)

    # draw mid of axle
    pygame.draw.circle(window, RED, (int(car_x), int(car_y)), 3)

    return rotated_corners


def draw_obstacles(window, blocks):
    obstacles = [x.get_pos()[1:] for x in blocks]
    [pygame.draw.rect(window, CUBE_COLORS[blocks[idx].orientation_id], x) for idx, x in enumerate(obstacles)]


def draw_grid(window, map_shape=(20, 20)):
    for i in range(1, map_shape[0]):
        curr_x = CUBE_WIDTH * i
        pygame.draw.line(window, WHITE, (curr_x, 0), (curr_x, WINDOW_HEIGHT))
    for i in range(1, map_shape[0]):
        curr_y = CUBE_LENGTH * i
        pygame.draw.line(window, WHITE, (0, curr_y), (WINDOW_WIDTH, curr_y))
