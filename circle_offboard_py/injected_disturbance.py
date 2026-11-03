"""L1Quad Section VI-A artificial matched thrust uncertainty (newtons)."""
import math


def thrust_disturbance(elapsed, enabled=True, start=2.0):
    """Use experiment time, not elapsed-start: preserves the paper's phase."""
    if not enabled or elapsed is None or elapsed < start:
        return 0.0
    return 2 * math.sin(2 * math.pi * elapsed) + 2 * math.sin(math.pi * elapsed)
