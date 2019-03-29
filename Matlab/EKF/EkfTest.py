import numpy as np
import pygame
from numpy import sin, cos, pi
from EKF import EKF
import time

# Define the colors we will use in RGB format
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# Set the height and width of the screen
size = [1200, 600]

# Define boolean for seeded random values
seeded = True

class Ekf:
    def __init__(self):
        self.current_state = np.zeros((5,1))

    def update(self):
        pass


# Generates wheel encoder data along a path
def generate_ticks():
    l = []
    r = []
    for i in range(5):
        l.append(10)
        r.append(9)
    for i in range(30):
        l.append(10)
        r.append(10)

    for i in range(40):
        l.append(9.5)
        r.append(10)

    for i in range(60):
        l.append(10)
        r.append(9.5)

    for i in range(20):
        l.append(9)
        r.append(10)

    for i in range(20):
        l.append(3)
        r.append(4)



    return l, r


class Robot:

    def __init__(self, param_screen, color = BLACK):
        self.screen = param_screen
        self.base_points = np.asarray([[0, 0], [0, 64], [5, 64], [5, 39], [27, 39],
                                       [27, 74], [34, 74], [34, 39], [56, 39], [56, 64],
                                       [61, 64], [61, 0], [56, 0], [56, 25], [5, 25], [5, 0]])
        self.base_points[:, 0] -= 30
        self.base_points[:, 1] -= 32
        self.robot_points = self.base_points.copy()
        self.old_positions = [[size[0]/2, size[1]]]
        self.x = size[0] / 2
        self.y = size[1]
        self.color = color

    def update_state(self, x, y, theta):
        self.old_positions.append([self.x, self.y])
        rot_mat = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        self.robot_points = np.matmul(self.base_points, rot_mat)

        self.robot_points[:, 0] += x + size[0] / 2
        self.robot_points[:, 1] += size[1] - y
        self.x = x + size[0] / 2
        self.y = size[1] - y





    def draw(self):
        # Should draw the robot
        pygame.draw.polygon(screen, self.color, list(self.robot_points))
        pygame.draw.lines(screen, self.color, False,  self.old_positions, 2)

class State:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def update(self, l, r, delta_t):
        v_l = l / delta_t
        v_r = r / delta_t

        # Theta gets updated first
        self.theta = self.theta + (v_r - v_l) / 57.15

        self.x += -.5 * (v_l + v_r)*delta_t * sin(self.theta)
        self.y += .5 * (v_l + v_r)*delta_t * cos(self.theta)


    def get_state(self):
        return self.x, self.y, self.theta


def draw():
    pass


if __name__ == '__main__':
    pygame.init()

    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("Extended Kalman Filter Test")
    clock = pygame.time.Clock()
    done = False


    robot = Robot(screen)
    ekf_robot = Robot(screen, BLUE)
    tracer_robot = Robot(screen, GREEN)


    x = 0
    y = 0

    # TODO: Generate path to be taken
    path_index = 0
    l,r = generate_ticks()
    #l = [1.5*x for x in l]
    #r = [1.5*x for x in r]
    delta_t = .1
    state = State()
    ekf_state = State()
    tracer_state = State()
    ekf = EKF()
    gps_readings = []

    if (seeded):
        np.random.seed(56462)
    while not done:
        # Clear the screen and set the screen background
        screen.fill(WHITE)

        # Turn off the simulation
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop

        #########################################################
        #                   Insert Animation Code Here
        #########################################################

        # Add some Gaussian noise to the current x,y and use that for GPS coords
        x_offset = np.random.normal(loc = 0, scale=20)
        y_offset = np.random.normal(loc = 0, scale=20)
        # Wheel Noise -- 3 Tick random variance
        l_offset = np.random.normal(loc = 0.1, scale=0.2)
        r_offset = np.random.normal(loc = 0.1, scale=0.2)
        #IMU Heading noise
        i_offset = np.random.normal(loc = 0, scale = 0.1)

        # Update the Ground Truth Values
        if path_index < len(l) - 1:
            path_index += 1
            imuHeading = state.theta + i_offset
            state.update(l[path_index], r[path_index], delta_t)
            tracer_state.update(l[path_index] + l_offset, r[path_index] + r_offset, delta_t)



            # Get the GPS readings
            gps_readings.append([state.get_state()[0] + x_offset + size[0] / 2 ,
                                 size[1] - (state.get_state()[1] + y_offset)])

            timestep = delta_t
            gpsX = state.get_state()[0] + x_offset
            gpsY = state.get_state()[1] + y_offset
            thetaR = r[path_index] + r_offset
            thetaL = l[path_index] + l_offset
            #print("Initial ThetaR:" + str(float(ekf.x[3])))
            #print("Initial ThetaL:" + str(float(ekf.x[4])))
            #print("GPS X: " + str(float(gpsX)))
            #print("GPS Y: " + str(float(gpsY)))
            ekf.step(timestep, gpsX, gpsY, imuHeading, thetaL, thetaR)
            #print("Updated ThetaR:" + str(float(ekf.x[3])))
            #print("Updated ThetaL:" + str(float(ekf.x[4])))
            #print("EKF X: " + str(float(ekf.x[0])))
            #print("EKF Y: " + str(float(ekf.x[1])))
            ekf_state.x = ekf.x[0]
            ekf_state.y = ekf.x[1]
            ekf_state.theta = ekf.x[2]

        robot.update_state(*(state.get_state()))
        tracer_robot.update_state(*(tracer_state.get_state()))
        ekf_robot.update_state(*(ekf_state.get_state()))

        # Draw the ground truth
        robot.draw()
        tracer_robot.draw()
        ekf_robot.draw()


        for reading in gps_readings:
            pygame.draw.circle(screen, RED, (int(reading[0]), int(reading[1])), 2)










        pygame.display.flip() # This must happen last
        time.sleep(delta_t)

