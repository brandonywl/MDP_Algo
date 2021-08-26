import numpy as np

from map_tiles.cell import Cell


class Robot(Cell):
    def __init__(self, theta, x, y, car_width, car_length):
        super().__init__(theta, x, y, 1, 1)
        self.cell_width = car_width
        self.cell_height = car_length
        self.car_length = car_length

        self.max_steering_angle = 45

        self.steering_angle = 0.0
        self.steering_drift = 0.0
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    def set(self, x, y, orientation, steering_angle):
        # if x >= world_size or x < 0:
        #     raise ValueError('X coordinate out of bound')
        # if y >= world_size or y < 0:
        #     raise ValueError('Y coordinate out of bound')
        # if orientation >= 2 * pi or orientation < 0:
        #     raise ValueError('Orientation must be in [0..2pi]')
        # if abs(steering_angle) > max_steering_angle:
        #     raise ValueError('Exceeding max steering angle')

        self.x = x
        self.y = y
        self.theta = orientation
        self.steering_angle = steering_angle

    def set_steering_drift(self, steering_drift):
        self.steering_drift = steering_drift

    def set_noise(self, f_noise, t_noise, s_noise):
        self.forward_noise = f_noise
        self.turn_noise = t_noise
        self.sense_noise = s_noise



    def move(self, turn, forward_velocity, frequency=1):
        """
        Moves a single timestep given turn and forward. How long a timestep is can be controlled with frequency.
        :param turn: Steering angle of the rear-wheeled vehicle.
        :param forward_velocity: Velocity of the car in pixel/instance or pixel/s.
        :param frequency: Controls delta time factor to convert to distance. If forward_velocity is provided in cm/s
        and converted to pixels/s, frequency will be 1/T where T is how long we want to stay in each command for the
        robot to move.
        :return: x,y, theta of the next state
        """

        theta = self.theta  # initial orientation
        alpha = turn  # steering angle
        dist = forward_velocity / frequency  # distance to be moved
        print(dist, forward_velocity, frequency)
        length = self.car_length  # length of the robot
        if abs(alpha) > self.max_steering_angle:
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
            cx = self.x - np.sin(theta) * radius  # center of the circle
            cy = self.y - np.cos(theta) * radius  # center of the circle

            # Uncomment to see the center of the circle the robot is going about
            # pygame.draw.circle(screen, red, (int(cx), int(cy)), 5)
            # pygame.display.update()

            # in global coordinates of robot
            _x = cx + np.sin(theta + beta) * radius
            _y = cy + np.cos(theta + beta) * radius
            _theta = (theta + beta) % (2 * np.pi)

        else:  # straight motion
            _x = self.x + dist * np.cos(theta)
            _y = self.y - dist * np.sin(theta)
            _theta = (theta + beta) % (2 * np.pi)

        self.x = _x
        self.y = _y
        self.theta = _theta
        self.steering_angle = alpha

        # self.x %= world_size
        # self.y %= world_size

    def move_forward(self, angle, speed):
        # Control Wheels

        # Control Motor

        # Update x, y, theta
        pass

    def image_rec(self):
        pass

    # @staticmethod
    # def get_new_xy(map_shape=(10, 10)):
    #     Block.get_new_xy(map_shape)
