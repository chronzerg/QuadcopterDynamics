class RotationMatrix:
    

def RotationMatrix(attitude):
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




def TransformationMatrix(attitude):
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


def AngularVelocity(attitudeRates, attitude):
    return TransformationMatrix(attitude)*attitudeRates


def AttitudeRates(angularVelocity, attitude):
    return TransformationMatrix(attitude).getI()*angularVelocity


def Thrust(inputs, k):
    """Thrust in the body frame."""
    xComponent = 0
    yComponent = 0
    zComponent = k*sum(inputs)

    return MakeVector((xComponent, yComponent, zComponent))


def Torque(inputs, L, b, k):
    """Torque in the body frame."""
    rollComponent = L*k*(inputs[0]-inputs[2])
    pitchComponent = L*k*(inputs[1]-inputs[3])
    yawComponent = b*(inputs[0]-inputs[1]+inputs[2]-inputs[3])

    return MakeVector((rollComponent, pitchComponent, yawComponent))


def LinearAcceleration(inputs, attitude, velocity, mass, g, k, kd):
    R = RotationMatrix(attitude)

    gravity = MakeVector((0, 0, -g))
    thrust = R*Thrust(inputs, k)
    drag = -kd*velocity

    return gravity+(thrust/mass)+drag


def AngularAcceleration(inputs, angularVelocity, inertia, L, b, k):
    torque = Torque(inputs, L, b, k)
    crossed = np.cross(angularVelocity, inertia*angularVelocity, axis=0)

    return inertia.getI()*(torque-crossed)