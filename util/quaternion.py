import numpy as np

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Quaternion({self.w} + {self.x}i + {self.y}j + {self.z}k)"
    

    def __add__(self, other):
        """Quaternion addition."""
        return Quaternion(
            self.w + other.w,
            self.x + other.x,
            self.y + other.y,
            self.z + other.z
        )
    

    def __mul__(self, other):
        """Quaternion multiplication."""
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        )
    

    def mul_by_scalar(self, scalar):
        """Multiply quaternion by a scalar."""
        return Quaternion(
            self.w * scalar,
            self.x * scalar,
            self.y * scalar,
            self.z * scalar
        )
    

    def get_norm(self):
        """Return the norm of the quaternion."""
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
    

    def normalize(self):
        """Return a normalized version of the quaternion."""
        norm = self.get_norm()
        if norm == 0:
            raise ValueError("Cannot normalize a quaternion with zero norm.")
        return self.mul_by_scalar(1.0 / norm)

    