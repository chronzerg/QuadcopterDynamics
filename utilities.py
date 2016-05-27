import numpy as np

def makeVector(items):
    """Converts a linear enumerable of items into a column vector."""
    return np.matrix(np.vstack(items))


def stringifyVector(vector):
    """Returns the 3 elements of a vector in a string separated by spaces."""
    return '%f %f %f' % tuple(vector.A.flatten())


def normalizeAngle(angle):
    while angle < 0:
        angle += 2*pi

    while angle > 2*pi:
        angle -= 2*pi

    return angle


def normalizeAngleVector(vector):
    for angle in np.nditer(vector, op_flags=['readwrite']):
        angle = normalize_angle(angle)

    return vector