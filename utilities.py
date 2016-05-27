import numpy as np
from pprint import pprint

def makeVector(items):
    """Converts a linear enumerable of items into a column vector."""
    return np.matrix(np.vstack(items))


def stringifyVector(vector):
    """Returns the 3 elements of a vector in a string separated by spaces."""
    return '%f %f %f' % tuple(vector.A.flatten())


def normalizeAngle(angle)
    """Normalize an angle between 0 and 360 degrees."""
    while angle < 0:
        angle += 2*pi

    while angle > 2*pi:
        angle -= 2*pi

    return angle


def normalizeAngleVector(vector):
    """Normalize a vector of angles between 0 and 360 degrees."""
    for angle in np.nditer(vector, op_flags=['readwrite']):
        angle = normalize_angle(angle)

    return vector


def printState(state):
    """Pretty print a physical model's state dictionary."""
    pprint(state)