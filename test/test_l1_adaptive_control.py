"""Check L1Quad signs, channels, prediction timing and clipped input feedback."""
import numpy as np
import pytest

from circle_offboard_py.l1_adaptive_control import L1AdaptiveControl


def law():
    return L1AdaptiveControl(2., np.diag([.02, .03, .04]))


def test_nominal_hover_and_first_observation():
    model = law()
    z, rotation = np.zeros(6), np.eye(3)
    sent = np.array([19.62, 0, 0, 0])
    for _ in range(100):
        assert np.allclose(model.step(z, rotation, sent, .01), 0)
    assert np.allclose(model.prediction_error, 0)


@pytest.mark.parametrize('jitter', [0., .0005])
@pytest.mark.parametrize('channel', range(6))
def test_disturbance_estimation_sign_units_and_unmatched_exclusion(channel, jitter):
    model = law()
    # A rotated frame verifies body/world conversions in both B and Bperp.
    angle = .4
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    b = model.input_matrix(rotation)
    disturbance = np.zeros(6)
    disturbance[channel] = .5 if channel in (0, 4, 5) else .015
    z = np.zeros(6)
    sent = np.array([19.62, 0., 0., 0.])
    adaptive = model.step(z, rotation, sent, .01)
    # Independent nominal plant, no feedback compensation, variable step size.
    for k in range(600):
        dt = .01 - jitter if k % 2 else .01 + jitter
        omega = z[3:]
        drift = np.r_[0, 0, 9.81,
                      -np.linalg.solve(model.inertia, np.cross(omega, model.inertia @ omega))]
        z += dt * (drift + b[:, :4] @ sent + b @ disturbance)
        adaptive = model.step(z, rotation, sent, dt)
    assert model.sigma[channel] / disturbance[channel] == pytest.approx(1., rel=.16)
    if channel < 4:
        assert adaptive[channel] * disturbance[channel] < 0
        assert adaptive[channel] == pytest.approx(-disturbance[channel], rel=.16)
    else:
        assert np.linalg.norm(adaptive) < 1e-8


def test_compensation_reduces_constant_thrust_disturbance():
    accelerations = []
    for apply in (False, True):
        model = law()
        z, rotation = np.zeros(6), np.eye(3)
        b = model.input_matrix(rotation)
        baseline = np.array([19.62, 0., 0., 0.])
        sent = baseline.copy()
        model.step(z, rotation, sent, .01)
        for _ in range(500):
            dz = model.drift(z) + b[:, :4] @ (sent + [1., 0, 0, 0])
            z += .01 * dz
            adaptive = model.step(z, rotation, sent, .01)
            sent = baseline + (adaptive if apply else 0)
        accelerations.append(abs(dz[2]))
    assert accelerations[1] < .15 * accelerations[0]


def test_prediction_uses_reconstructed_nonlinear_clipped_input():
    from circle_offboard_py.actuator_mapping import X500ActuatorMap
    model = law()
    normalized, sent = X500ActuatorMap().map(100., [4., -5., .5])
    assert sent[0] < 100.
    assert np.all(np.isfinite(normalized))
    z, rotation, dt = np.zeros(6), np.eye(3), .01
    model.step(z, rotation, np.zeros(4), dt)
    z += dt * (model.drift(z) + model.input_matrix(rotation)[:, :4] @ sent)
    model.step(z, rotation, sent, dt)
    assert np.allclose(model.sigma, 0, atol=1e-10)


def test_reset_discards_estimation_and_filter_history():
    model = law()
    model.step(np.zeros(6), np.eye(3), np.zeros(4), .01)
    model.step(np.ones(6), np.eye(3), np.zeros(4), .01)
    assert np.linalg.norm(model.sigma) > 0
    model.reset()
    assert np.allclose(model.step(np.ones(6), np.eye(3), np.zeros(4), .01), 0)
    assert np.allclose(model.sigma, 0)


@pytest.mark.parametrize('dt', [0, -.01, np.nan, .1])
def test_invalid_intervals_rejected(dt):
    with pytest.raises(ValueError):
        law().step(np.zeros(6), np.eye(3), np.zeros(4), dt)


def test_structured_input_inverse_matches_dense_solve_for_full_inertia():
    model = L1AdaptiveControl(2., np.array([[.03, .002, .001],
                                           [.002, .04, -.001], [.001, -.001, .05]]))
    rng = np.random.default_rng(77)
    for _ in range(30):
        q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
        if np.linalg.det(q) < 0:
            q[:, 0] *= -1
        value = rng.normal(size=6)
        assert np.allclose(model.solve_input_matrix(q, value),
                           np.linalg.solve(model.input_matrix(q), value), atol=1e-12)
