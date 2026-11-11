import numpy as np
from util.quaternion import Quaternion


class Rigidbody:
    # kg, cm, cm
    def __init__(self, mass, radius, length):
        # cylinder def
        self.mass = mass
        self.radius = radius
        self.length = length
        self.moi = self.calc_moi(mass, radius, length)

        # pos
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
        self.angular_velocity = np.zeros(3)

    def calc_moi(mass, radius, length):
        return np.array([
            [(mass/12)*(3 * radius**2 + length**2), 0, 0],
            [0, (mass/12)*(3 * radius**2 + length**2), 0],
            [0, 0, 0.5 * mass * radius**2]
            ])

