import numpy as np
from math import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


def CalculateNextFrame(position, positionRates, attitude, attitudeRates):
    """Calculate the next frame of the simulation."""
    c = constants

    angularVelocity = AttitudeRatesToAngularVelocity(attitudeRates, attitude)

    acceleration = CalculateLinearAcceleration(inputs, attitude, positionRates,
                                               c['mass'], c['g'], c['k'],
                                               c['kd'])
    angularAcceleration = CalculateAngularAcceleration(inputs, angularVelocity,
                                                       c['inertia'], c['L'],
                                                       c['b'], c['k'])

    angularVelocity = angularVelocity + (dt * angularAcceleration)
    attitudeRates = AngularVelocityToAttitudeRates(angularVelocity, attitude)
    attitude = NormalizeAngleVector(attitude + (dt * attitudeRates))

    positionRates = positionRates + (dt * acceleration)
    position = position + (dt * positionRates)

    return position, positionRates, attitude, attitudeRates


def ValidState(position):
    if position[2, 0] <= 0:
        return False
    else:
        return True


def RunSimulationTick():
    global position, positionRates, attitude, attitudeRates

    if dontCheckState or ValidState(position):
        position, positionRates, attitude, attitudeRates = \
            CalculateNextFrame(position, positionRates, attitude,
                               attitudeRates)
        # TODO: Calculate controller input
        PrintState()
        DrawFrame()

# Main
verbose = False
dontCheckState = True
slowFactor = 100
dt = 0.050
margins = 1


# Constants
constants = dict()
constants['g'] = 9.81  # m/s
constants['mass'] = 1  # kg
constants['L'] = 0.37  # m

# Assumming the quadcopter is 4 point masses at the motors...
# Inertias in units of kg*(m^2)
Ixx = 2*(constants['mass']/4)*pow(constants['L'], 2)
Iyy = Ixx
Izz = 2*Ixx

constants['k'] = 1  # thrust constant
constants['kd'] = 1  # drag constant
constants['b'] = 1  # torque constant
constants['inertia'] = np.matrix(np.diag((Ixx, Iyy, Izz)))  # interial matrix


# Physical State
position = MakeVector((0, 0, 10))
positionRates = MakeVector((0, 0, 0))
attitude = MakeVector((0, 0, 0))
attitudeRates = MakeVector((0.1, 0.2, 0.3))
angularVelocity = MakeVector((0, 0, 0))


# Controller
inputs = [0, 0, 0, 0]


# Plotting
# xArm and yArm are body-frame vectors representing the quadcopter's arms in
# the x and y directions.
xArm = MakeVector((constants['L'], 0, 0))
yArm = MakeVector((0, constants['L'], 0))

# The point used to draw the "up" direction vector from the top of the forward
# facing x arm.
upArm = MakeVector((constants['L'], 0, 0.1))

figure = plt.figure()
axes = p3.Axes3D(figure)
xLine = Create3dLine(axes, dict(marker='o', markersize=7, color='c', linewidth=5))
yLine = Create3dLine(axes, dict(marker='o', markersize=7, color='c', linewidth=5))
upLine = Create3dLine(axes, dict(marker='o', markersize=5, markevery=[1], color='r', linewidth=2))

line_ani = animation.FuncAnimation(figure, RunSimulationTick, None, interval=50, blit=False)

plt.show()
