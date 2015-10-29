import numpy as np
from math import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


"""Converts a linear enumerable of items into a column vector."""
def MakeVector(items):
	return np.matrix(np.vstack(items))

"""Returns the 3 elements of a vector in a string separated by spaces."""
def StringifyVector(vector):
	return '%f %f %f' % tuple(vector.A.flatten())


"""Prints the position, position rates, attitude, and attitude rates."""
def PrintState():
	print("Pos:  ", StringifyVector(position))
	print("Pos*: ", StringifyVector(positionRates))
	print("Att:  ", StringifyVector(attitude))
	print("Att*: ", StringifyVector(attitudeRates), "\n")


"""R is the rotation matrix for converting vectors from the body frame to the inertial frame."""
def CalculateR(attitude):
	roll =  attitude[0,0]
	pitch = attitude[1,0]
	yaw =   attitude[2,0]

	R = np.matrix(np.zeros((3,3)))

	R[0,0] = (cos(roll)*cos(yaw)) - (cos(pitch)*sin(roll)*sin(yaw))
	R[0,1] = -(cos(yaw)*sin(roll)) - (cos(roll)*cos(pitch)*sin(yaw))
	R[0,2] = sin(pitch)*sin(yaw)

	R[1,0] = (cos(pitch)*cos(yaw)*sin(roll)) + (cos(roll)*sin(yaw))
	R[1,1] = (cos(roll)*cos(pitch)*cos(yaw)) - (sin(roll)*sin(yaw))
	R[1,2] = -(cos(yaw)*sin(pitch))

	R[2,0] = sin(roll)*sin(pitch)
	R[2,1] = cos(roll)*sin(pitch)
	R[2,2] = cos(pitch)

	return R


"""T is the transformation matrix for converting a vector of euler angle velocities to an angular velocity vector (about which the body rotates)."""
def CalculateT(attitude):
	roll =  attitude[0,0]
	pitch = attitude[1,0]
	yaw =   attitude[2,0]

	T = np.matrix(np.zeros((3,3)))

	T[0,0] = 1
	T[0,1] = 0
	T[0,2] = -sin(pitch)

	T[1,0] = 0
	T[1,1] = cos(roll)
	T[1,2] = cos(pitch)*sin(roll)

	T[2,0] = 0
	T[2,1] = -sin(roll)
	T[2,2] = cos(pitch)*cos(roll)

	return T


def AttitudeRatesToAngularVelocity(attitudeRates, attitude):
	return CalculateT(attitude)*attitudeRates


def AngularVelocityToAttitudeRates(angularVelocity, attitude):
	return CalculateT(attitude).getI()*angularVelocity


"""Thrust in the body frame."""
def CalculateThrust(inputs, k):
	xComponent = 0
	yComponent = 0
	zComponent = k*sum(inputs)

	return MakeVector((xComponent, yComponent, zComponent))


"""Torque in the body frame."""
def CalculateTorque(inputs, L, b, k):
	rollComponent =  L*k*(inputs[0]-inputs[2])
	pitchComponent = L*k*(inputs[1]-inputs[3])
	yawComponent =   b*(inputs[0]-inputs[1]+inputs[2]-inputs[3])

	return MakeVector((rollComponent, pitchComponent, yawComponent))


def CalculateLinearAcceleration(inputs, attitude, velocity, mass, g, k, kd):
	R = CalculateR(attitude)

	gravity = MakeVector((0, 0, -g))	
	thrust =  R*CalculateThrust(inputs, k)
	drag =    -kd*velocity
	
	return gravity+(thrust/mass)+drag


def CalculateAngularAcceleration(inputs, angularVelocity, inertia, L, b, k):
	torque = CalculateTorque(inputs, L, b, k)

	return inertia.getI()*(torque-np.cross(angularVelocity, inertia*angularVelocity, axis=0))


"""Draw the current frame of the simulation onto the plot."""
def DrawFrame():
	R = CalculateR(attitude)

	# Transform the arms into the inertia frame and store them as a flattened list.
	px = (R * xArm).A.flatten()
	py = (R * yArm).A.flatten()

	x = position[0,0]
	y = position[1,0]
	z = position[2,0]

	# Plot both arms in the body-frame's x axis.
	xLine.set_data((x-px[0], x+px[0]), (y-px[1], y+px[1]))
	xLine.set_3d_properties((z-px[2], z+px[2]))

	# Plot both arms in the body-frame's y axis.
	yLine.set_data((x-py[0], x+py[0]), (y-py[1], y+py[1]))
	yLine.set_3d_properties((z-py[2], z+py[2]))

	axes.set_xlim(x-margins, x+margins)
	axes.set_ylim(y-margins, y+margins)
	axes.set_zbound(z-margins, z+margins)


"""Calculate the next frame of the simulation."""
def CalculateNextFrame(position, positionRates, attitude, attitudeRates):
	c = constants

	angularVelocity = AttitudeRatesToAngularVelocity(attitudeRates, attitude)

	acceleration = CalculateLinearAcceleration(inputs, attitude, positionRates, c['mass'], c['g'], c['k'], c['kd'])
	angularAcceleration = CalculateAngularAcceleration(inputs, angularVelocity, c['inertia'], c['L'], c['b'], c['k'])

	angularVelocity = angularVelocity + (dt * angularAcceleration)
	attitudeRates = AngularVelocityToAttitudeRates(angularVelocity, attitude)
	attitude = attitude + (dt * attitudeRates)

	positionRates = positionRates + (dt * acceleration)
	position = position + (dt * positionRates)

	return position, positionRates, attitude, attitudeRates


def RunSimulationTick(num):
	global position, positionRates, attitude, attitudeRates
	position, positionRates, attitude, attitudeRates = CalculateNextFrame(position, positionRates, attitude, attitudeRates)
	# TODO: Calculate controller input
	DrawFrame()



              ##############
################## Main ##################
              ##############

verbose = False
slowFactor = 100
dt = 0.01
margins = 1


##### Constants ##########################
constants = dict()
constants['g'] = 9.81 #m/s
constants['mass'] = 1 #kg
constants['L'] = 0.37 #m

# Assumming the quadcopter is 4 point masses at the motors...
# Inertias in units of kg*(m^2)
Ixx = 2*(constants['mass']/4)*pow(constants['L'],2)
Iyy = Ixx
Izz = 2*Ixx

constants['k'] = 1 #thrust constant
constants['kd'] = 1 #drag constant
constants['b'] = 1 #torque constant
constants['inertia'] = np.matrix(np.diag((Ixx, Iyy, Izz))) #interial matrix


##### Physical State #####################
position = MakeVector((0, 0, 10))
positionRates = MakeVector((0, 0, 0))
attitude = MakeVector((0, 0, 0))
attitudeRates = MakeVector((1, 1, 0))


##### Controller #########################
inputs = [0, 0, 0, 0]


##### Plotting ###########################
# xArm and yArm are body-frame vectors representing the quadcopter's arms in the x and y directions.
xArm = MakeVector((constants['L'], 0, 0))
yArm = MakeVector((0, constants['L'], 0))

figure = plt.figure()
axes = p3.Axes3D(figure)
xLine = axes.plot((-1,1),(0,0),(0,0))[0]
yLine = axes.plot((0,0),(-1,1),(0,0))[0]

line_ani = animation.FuncAnimation(figure, RunSimulationTick, None, interval=25, blit=False)

plt.show()