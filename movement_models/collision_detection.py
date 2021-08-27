from typing import List

from utilities import drawing


def has_collision(robot_point, obstacles):
    robot_corners = drawing.get_robot_corners(robot_point.x, robot_point.y, robot_point.theta)

    obstacles_corners = [drawing.get_block_corners(block) for block in obstacles]

    for i, obstacle in enumerate(obstacles_corners):
        if is_collided(robot_corners, obstacle):
            print("Collision detected")
            return True
    return False


# Detects collision between two objects where corners of each lists are listed in clockwise direction
# Implementation of Separation Axis Theorem
# References: https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles
def is_collided(obj_a: List, obj_b: List):
    polygons = [obj_a, obj_b]
    for polygon in polygons:
        for i1 in range(len(polygon)):
            i2 = (i1 + 1) % len(polygon)
            p1 = polygon[i1]
            p2 = polygon[i2]

            normal = [p2[1] - p1[1], p1[0] - p2[0]]

            min_a, max_a = check_projection(normal, obj_a)
            min_b, max_b = check_projection(normal, obj_b)
            if max_a < min_b or max_b < min_a:
                return False
    return True


def check_projection(normal, points):
    min_val, max_val = None, None
    for point in points:
        projected = normal[0] * point[0] + normal[1] * point[1]
        if min_val is None or projected < min_val:
            min_val = projected
        if max_val is None or projected > max_val:
            max_val = projected

    return min_val, max_val


if __name__ == '__main__':
    # False test-case
    square_1 = [[1, 1], [2, 1], [2, 2], [1, 2]]
    square_2 = [[3, 1], [4, 1], [4, 2], [3, 2]]
    # Should be false
    print(is_collided(square_1, square_2))

    # True test-case
    square_3 = [[2, 1], [3, 1], [3, 2], [2, 2]]
    # Should be true
    print(is_collided(square_1, square_3))

    # Diamond test-case (True)
    diamond_1 = [[2, 1.5], [2.7, 0.8], [3.4, 1.5], [2.7, 2.2]]
    print(is_collided(square_1, diamond_1))

    # Diamond test-case (False)
    diamond_2 = [[2.01, 1.5], [2.7, 0.8], [3.4, 1.5], [2.7, 2.2]]
    print(is_collided(square_1, diamond_2))

    # Double-diamond test-case (True)
    diamond_3 = [[0.5, 1], [1, 0], [1.5, 1], [1, 2]]
    diamond_4 = [[1.5, 1], [2, 0], [2.5, 1], [2, 2]]
    print(is_collided(diamond_3, diamond_4))

    # Double-diamond test-case (True)
    diamond_5 = [[0.5, 1], [1, 0], [1.5, 1], [1, 2]]
    diamond_6 = [[1, 1], [1.5, 0], [2, 1], [1.5, 2]]
    print(is_collided(diamond_5, diamond_6))

    # Different number of points (True)
    house_1 = [[0, 0], [1, 0], [1, 1], [0.5, 1.5], [0, 1]]
    line_1 = [[0.5, 0.5], [1.5, 0.5]]
    print(is_collided(house_1, line_1))

    # Different number of points (False)
    house_2 = [[0, 0], [1, 0], [1, 1], [0.5, 1.5], [0, 1]]
    line_2 = [[2.5, 0.5], [3.5, 0.5]]
    print(is_collided(house_2, line_2))

    test_1 = [(-30.68679617103011, 611.4364224790827), (-24.922199081546182, 551.7139859882972), (34.800237409239365, 557.4785830777811), (29.03564031975545, 617.2010195685666)]
    test_2 = [[270, 570], [300, 570], [300, 600], [270, 600]]
    print(is_collided(test_1, test_2))