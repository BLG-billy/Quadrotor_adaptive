"""Paper disturbance timing, phase, and input separation."""
import math

import numpy as np
import pytest

from circle_offboard_py.injected_disturbance import thrust_disturbance
from circle_offboard_py.l1_adaptive_control import L1AdaptiveControl


def test_disturbance_gate_and_absolute_phase():
    assert thrust_disturbance(None) == 0
    assert thrust_disturbance(1.99) == 0
    assert thrust_disturbance(2.25, False) == 0
    assert thrust_disturbance(2.25) == pytest.approx(2. + 2. / math.sqrt(2))
    assert thrust_disturbance(3.25) == pytest.approx(2. - 2. / math.sqrt(2))


def test_l1_estimates_unknown_input_while_second_filter_converges():
    model = L1AdaptiveControl(2., np.diag([.02, .03, .04]),
                              cutoff=[30., 5., 5., 5.],
                              moment_cutoff_second=[15., 15., 15.])
    z, rotation = np.zeros(6), np.eye(3)
    nominal = np.array([19.62, 0., 0., 0.])
    b = model.input_matrix(rotation)
    disturbance = np.array([.6, .01, -.01, .005])
    model.step(z, rotation, nominal, .01)
    for _ in range(600):
        z += .01 * (model.drift(z) + b[:, :4] @ (nominal + disturbance))
        adaptive = model.step(z, rotation, nominal, .01)
    assert np.allclose(adaptive, -disturbance, rtol=.16, atol=.001)
    model.reset()
    assert np.allclose(model.filtered_second, 0)
