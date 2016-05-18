def MakeVector(items):
    """Converts a linear enumerable of items into a column vector."""
    return np.matrix(np.vstack(items))


def StringifyVector(vector):
    """Returns the 3 elements of a vector in a string separated by
    spaces."""
    return '%f %f %f' % tuple(vector.A.flatten())


def PrintState():
    """Prints the position, position rates, attitude, and attitude
    rates."""
    print("Pos:  ", StringifyVector(position))
    print("Pos*: ", StringifyVector(positionRates))
    print("Att:  ", StringifyVector(attitude))
    print("Att*: ", StringifyVector(attitudeRates), "\n")


def NormalizeAngle(angle):
    while angle < 0:
        angle += 2*pi

    while angle > 2*pi:
        angle -= 2*pi

    return angle


def NormalizeAngleVector(vector):
    for angle in np.nditer(vector, op_flags=['readwrite']):
        angle = NormalizeAngle(angle)

    return vector