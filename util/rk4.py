import numpy as np

def rk4_step(f, y0: np.ndarray, t0: float, dt: float) -> np.ndarray:
    """Perform a single step in RK4 integration, returning the new value of y."""
    k1 = f(t0, y0)
    k2 = f(t0 + dt / 2, y0 + dt * k1 / 2)
    k3 = f(t0 + dt / 2, y0 + dt * k2 / 2)
    k4 = f(t0 + dt, y0 + dt * k3)

    return y0 + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)