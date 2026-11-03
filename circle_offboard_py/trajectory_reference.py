"""Analytic C4 takeoff and circle reference, independent of ROS.

Atomic wire format on /circle/reference_high_order (Float64MultiArray):
[timestamp_us, p(3), v(3), a(3), jerk(3), snap(3), yaw, yaw_rate, yaw_accel].
All vectors use NED; yaw uses radians, derivatives use seconds.
"""
import math
from types import SimpleNamespace

import numpy as np


def polynomial(coefficients, t, derivative=0):
    """Evaluate an ascending-power polynomial or an exact derivative."""
    return sum(c * math.factorial(i) / math.factorial(i - derivative)
               * t**(i - derivative)
               for i, c in enumerate(coefficients) if i >= derivative)


def circle_reference(t, radius, omega, ramp):
    """Return p/v/a/jerk/snap with zero derivatives through snap at entry.

    Angular speed uses a seventh-degree smoothstep, with its first three
    derivatives zero at both ends; its integral determines the angle.
    """
    if ramp > 0 and t < ramp:
        u = max(0., t / ramp)
        speed_poly = [0, 0, 0, 0, 35, -84, 70, -20]
        integral = [0] + [c / (i + 1) for i, c in enumerate(speed_poly)]
        angle = omega * ramp * polynomial(integral, u)
        w, alpha, beta, gamma = [
            omega * polynomial(speed_poly, u, d) / ramp**d for d in range(4)]
    else:
        angle = omega * (t - .5 * ramp)
        w, alpha, beta, gamma = omega, 0., 0., 0.
    # Complex plane provides the exact derivatives of r*exp(i*angle).
    z = radius * np.exp(1j * angle)
    derivatives = [z - radius, 1j * w * z, (1j * alpha - w * w) * z,
                   (1j * beta - 3 * w * alpha - 1j * w**3) * z,
                   (w**4 - 3 * alpha**2 - 4 * w * beta
                    + 1j * (gamma - 6 * w * w * alpha)) * z]
    return [np.array([value.real, value.imag, 0.]) for value in derivatives]


def takeoff_reference(t, height, duration):
    """Ninth-degree position blend: derivatives 1..4 vanish at endpoints."""
    if duration <= 0 or t >= duration:
        return [np.array([0., 0., -height])] + [np.zeros(3) for _ in range(4)]
    coefficients = [0, 0, 0, 0, 0, 126, -420, 540, -315, 70]
    u = max(0., t / duration)
    return [np.array([0., 0., -height * polynomial(coefficients, u, d) / duration**d])
            for d in range(5)]


def unpack_reference(data):
    """Validate one atomic reference packet before exposing any field."""
    values = np.asarray(data, dtype=float)
    if values.shape != (19,) or not np.isfinite(values).all():
        raise ValueError('Expected 19 finite high-order reference values')
    return SimpleNamespace(timestamp=int(values[0]), position=values[1:4],
                           velocity=values[4:7], acceleration=values[7:10],
                           jerk=values[10:13], snap=values[13:16],
                           yaw=values[16], yawspeed=values[17], yawaccel=values[18])
