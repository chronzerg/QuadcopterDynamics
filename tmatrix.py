import numpy as np

class Matrix:
    _matrix = None
    _attitude = None

    def _calculate(self, attitude):
        """The transformation matrix for converting a vector of euler
        angle velocities to an angular velocity vector (about which the
        body rotates)."""
        roll = attitude[0, 0]
        pitch = attitude[1, 0]

        T = np.matrix(np.zeros((3, 3)))

        T[0, 0] = 1
        T[0, 1] = 0
        T[0, 2] = -sin(pitch)

        T[1, 0] = 0
        T[1, 1] = cos(roll)
        T[1, 2] = cos(pitch)*sin(roll)

        T[2, 0] = 0
        T[2, 1] = -sin(roll)
        T[2, 2] = cos(pitch)*cos(roll)

        return T

    def get(self, attitude):
        """Get a copy of the rotation matrix, updating the cache if necessary."""
        if not np.array_equal(attitude, _attitude):
            _attitude = attitude
            _matrix = self._calculate(attitude)

        return _matrix