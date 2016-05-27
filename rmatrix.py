import numpy as np

class Matrix:
    _matrix = None
    _attitude = None

    def _calculate(self, attitude):
        """The rotation matrix for converting vectors from the body
        frame to the inertial frame."""
        roll = attitude[0, 0]
        pitch = attitude[1, 0]
        yaw = attitude[2, 0]

        R = np.matrix(np.zeros((3, 3)))

        R[0, 0] = (cos(roll)*cos(yaw)) - (cos(pitch)*sin(roll)*sin(yaw))
        R[0, 1] = -(cos(yaw)*sin(roll)) - (cos(roll)*cos(pitch)*sin(yaw))
        R[0, 2] = sin(pitch)*sin(yaw)

        R[1, 0] = (cos(pitch)*cos(yaw)*sin(roll)) + (cos(roll)*sin(yaw))
        R[1, 1] = (cos(roll)*cos(pitch)*cos(yaw)) - (sin(roll)*sin(yaw))
        R[1, 2] = -(cos(yaw)*sin(pitch))

        R[2, 0] = sin(roll)*sin(pitch)
        R[2, 1] = cos(roll)*sin(pitch)
        R[2, 2] = cos(pitch)

        return R

    def get(self, attitude):
        """Get a copy of the rotation matrix, updating the cache if necessary."""
        if not np.array_equal(attitude, _attitude):
            _attitude = attitude
            _matrix = self._calculate(attitude)

        return _matrix