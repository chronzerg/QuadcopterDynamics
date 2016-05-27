"""
Physical Models
"""

from tmatrix import Matrix as TMatrix
from rmatrix import Matrix as RMatrix
import utilities as ut
import numpy as np

class QuadCopter:
    _defaultConstants = {
        # Acceleration of gravity in m/(s^2)
        'g': 9.81,

        # Mass in kg
        'mass': 1,

        # Length of each arm in m
        'armLength': 0.25,

        # Angular mass in kg*(m^2)
        'intertia': 1,

        # Thrust multiplier. We simplify the thrust calculation
        # for each motor by making it a multiple of the propeller's
        # angular velocity.
        'thrust': 1,

        # Torque multiplier. We simplify the torque calculation
        # for each motor by making it a multiple of the propeller's
        # angular velocity.
        'torque': 1,

        # Drag multiplier. We simplify the drag calculation by
        # making it a multiple of the linear velocity of the
        # copter.
        'drag': 1
    }

    _defaultInits = {
        # X, y, and z.
        'position': ut.makeVector((0, 0, 10)),

        # Roll, pitch, and yaw angles. TODO - what frame?
        'attitude': ut.makeVector((0, 0, 0)),

        # The rate of change for each of the state vectors.
        'positionRates': ut.makeVector((0, 0, 0)),
        'attitudeRates': ut.makeVector((0, 0, 0)),

        # The inputs to each motor, representing the target
        # angular velocity.
        'inputs': [0, 0, 0, 0]
    }

    def __init__(self, constants, initials):
        self.r = RMatrix()
        self.t = TMatrix()

        self.c = self._defaultConstants.copy()
        self.c.update(constants)

        self.state = self._defaultInits.copy()
        self.state.update(initials)


    def setInputs(motor, value, values=None):
        """Sets the input value for one or all the motors."""
        if values is not None:
            self.state['inputs'] = values
        else:
            self.state['inputs'][motor] = value


    def updateState(elapsedTime):
        angularVelocity = self._calcAngularVelocity()

        acceleration = self._calcLinearAcceleration()
        angularAcceleration = self._calcAngularAcceleration()

        angularVelocity = angularVelocity + (elapsedTime * angularAcceleration)
        attitudeRates = self._calcAttitudeRates(angularVelocity)
        attitude = ut.normalizeAngleVector(attitude + (elapsedTime * attitudeRates))

        positionRates = positionRates + (elapsedTime * acceleration)
        position = position + (elapsedTime * positionRates)

        self.state.update({
            'position': position,
            'attitude': attitude,
            'positionRates': positionRates,
            'attitudeRates': attitudeRates
        })


    def _calcAngularVelocity(self):
        """Angular velocity in the world frame."""
        return self.t.get(self.attitude)*self.attitudeRates


    def _calcAttitudeRates(self, angularVelocity):
        """Attitude rates in the body frame."""
        return self.t.get(self.attitude).getI()*angularVelocity


    def _calcThrust(self):
        """Thrust in the body frame."""
        xComponent = 0
        yComponent = 0
        zComponent = self.c['thrust']*sum(self.state['inputs'])

        return ut.makeVector((xComponent, yComponent, zComponent))


    def _calcTorque(self):
        """Torque in the body frame."""
        inputs = self.state['inputs']
        l = self.c['armLength']
        th = self.c['thrust']
        tq = self.c['torque']

        rollComponent = l*th*(inputs[0]-inputs[2])
        pitchComponent = l*th*(inputs[1]-inputs[3])
        yawComponent = tq*(inputs[0]-inputs[1]+inputs[2]-inputs[3])

        return ut.makeVector((rollComponent, pitchComponent, yawComponent))


    def _calcLinearAcceleration(self):
        """Linear acceleration in the world frame."""
        r = self.r.get(self.attitude)
        gravity = ut.makeVector((0, 0, -self.c['g']))
        thrust = r*self._calcThrust()
        drag = -self.c['drag']*self.state['positionRates']

        return gravity+(thrust/self.c['mass'])+drag


    def _calcAngularAcceleration(self):
        """Angular acceleration in the world frame."""
        torque = self._calcTorque()
        angularVelocity = self._calcAngularVelocity()
        crossed = np.cross(angularVelocity, self.c['intertia']*angularVelocity, axis=0)

        return inertia.getI()*(torque-crossed)