import numpy as np

g = 9.81  # m/s
mass = 1  # kg
armLength = 0.37 # m

# Assumming the quadcopter is 4 point masses at the motors...
# Inertias in units of kg*(m^2)
_inertiaXX = 2*(mass/4)*pow(armLength, 2)
_inertiaYY = inertiaXX
_inertiaZZ = 2*inertiaXX
inertia = np.matrix(np.diag((inertiaXX, inertiaYY, inertiaZZ)))

# Multipliers used as a super simple way to approxiate certain characteristics.
thrust = 1
torque = 1
drag = 1

dictionary = globals()