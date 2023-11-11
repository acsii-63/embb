import numpy as np
from pyquaternion import Quaternion

v = np.array([0.0,-1.0,0.0])
q = Quaternion(axis=[1, -1, 0], angle=-np.pi)
vr = q.rotate(v)

# Extract the rotated vector
np.set_printoptions(precision=2, suppress=True)
print("Original Vector:", v)
print("Rotated Vector:", vr)