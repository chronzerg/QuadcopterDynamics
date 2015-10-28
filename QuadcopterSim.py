import numpy as np
from math import *
import time


"""Converts a linear enumerable of items into a column vector."""
def MakeVector(items):
	return np.matrix(np.vstack(items))


def StringifyVector(vector):
	return '%f %f %f' % tuple(vector.A.flatten())


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


## Main ##

dt = 0.005 #ms
slowFactor = 1

g = 9.81 #m/s

mass = 1 #kg
inertia = np.matrix(np.diag((1,1,1))) #TODO: get unit
L = 0.37 #m

k = 1 #thrust constant
kd = 1 #drag constant
b = 1 #torque constant

position = MakeVector((0, 0, 10))
positionRates = MakeVector((0, 0, 0))

attitude = MakeVector((0, 0, 0))
attitudeRates = MakeVector((2, 1, 0))

print("Pos:  ", StringifyVector(position))
print("Pos*: ", StringifyVector(positionRates))
print("Att:  ", StringifyVector(attitude))
print("Att*: ", StringifyVector(attitudeRates), "\n")

while True:
	# TODO: Calculate input
	inputs = [0, 0, 0, 0]

	angularVelocity = AttitudeRatesToAngularVelocity(attitudeRates, attitude)

	acceleration = CalculateLinearAcceleration(inputs, attitude, positionRates, mass, g, k, kd)
	angularAcceleration = CalculateAngularAcceleration(inputs, angularVelocity, inertia, L, b, k)

	angularVelocity = angularVelocity + (dt * angularAcceleration)
	attitudeRates = AngularVelocityToAttitudeRates(angularVelocity, attitude)
	attitude = attitude + (dt * attitudeRates)

	positionRates = positionRates + (dt * acceleration)
	position = position + (dt * positionRates)

	print("Pos:  ", StringifyVector(position))
	print("Pos*: ", StringifyVector(positionRates))
	print("Att:  ", StringifyVector(attitude))
	print("Att*: ", StringifyVector(attitudeRates), "\n")

	time.sleep(dt * slowFactor)