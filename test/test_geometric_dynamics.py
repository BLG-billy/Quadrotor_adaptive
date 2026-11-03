"""Closed-loop regression in an independent rigid-body/motor-lag plant.

This is deliberately not a PX4/Gazebo flight test. It exercises the physical
control/allocation chain with quantized observations at 100 Hz and a plant
integrated at 400 Hz, including unequal motor rise/fall time constants.
"""
import numpy as np
from circle_offboard_py.geometric_control import GeometricControlLaw
from circle_offboard_py.trajectory_reference import circle_reference
from circle_offboard_py.actuator_mapping import X500ActuatorMap


def simulate():
    c = GeometricControlLaw()
    c.mass, c.gravity, c.yaw = 2.0643076923076924, 9.81, 0.
    c.inertia = np.diag([.02166666666666667, .02166666666666667, .04])
    # Exercise the mass/inertia-normalized default design in a separate plant.
    c.kx = c.mass * np.array([2.5, 2.5, 2.5])**2
    c.kv = 2 * c.mass * np.array([2.5, 2.5, 2.5])
    c.kR = np.diag(c.inertia) * np.array([10., 10., 6.])**2
    c.kOmega = 2 * .85 * np.diag(c.inertia) * np.array([10., 10., 6.])
    c.max_commanded_rate, c.max_commanded_acceleration = 3., 15.
    c.previous_rc = c.previous_omega_c = None
    mapper = X500ActuatorMap()
    position, velocity, rotation, omega = np.array(
        [0.,0.,-1.]), np.zeros(3), np.eye(3), np.zeros(3)
    motor = np.full(4, mapper.collective(c.mass * c.gravity))
    command = motor.copy()
    errors, moment_values = [], []
    step = .0025
    for k in range(4800):
        t = k * step
        target_time = max(0.,t - .5)
        p,v,a,j,s = circle_reference(target_time,1.,.8,2.)
        p[2] = -1.
        if k%4 == 0:
            c.position = position.copy()
            # Held/quantized velocity approximates sampling discontinuities;
            # this is a synthetic plant, not PX4/Gazebo or a flight-log replay.
            c.velocity = np.round(velocity / .02) * .02
            c.rotation, c.angular_velocity = rotation.copy(), omega.copy()
            c.reference_jerk,c.reference_snap = j,s
            f,m,_ = c.geometric_control(p,v,a,.01)
            normalized,_ = mapper.map(f,m)
            command = np.clip(mapper.mixer @ normalized,0,1)
            moment_values.append(m)
        tau = np.where(command > motor,.0125,.025)
        motor += (command - motor) * (-np.expm1(-step / tau))
        wrench = mapper.force_matrix @ mapper.rotor_force(motor)
        acceleration = np.array([0,0,c.gravity]) - wrench[0] / c.mass * rotation[:,2]
        angular_acceleration = np.linalg.solve(
            c.inertia,wrench[1:] - np.cross(omega,c.inertia @ omega))
        position += step * velocity + .5 * step**2 * acceleration
        velocity += step * acceleration
        angle = omega * step + .5 * angular_acceleration * step**2
        theta = np.linalg.norm(angle)
        hat = c.hat(angle)
        if theta > 1e-12:
            rotation = rotation @ (np.eye(3) + np.sin(theta) / theta * hat + \
                                (1 - np.cos(theta)) / theta**2 * hat @ hat)
        omega += step * angular_acceleration
        if t > 3.:
            errors.append(np.linalg.norm(position - p))
    return dict(position_rms=float(np.sqrt(np.mean(np.square(errors)))),
                requested_moment_rms=np.sqrt(np.mean(np.square(moment_values),axis=0)).tolist(),
                max_angular_velocity=float(np.max(np.abs(omega))))

def test_circle_tracking_with_motor_lag_and_quantized_velocity():
    result = simulate()
    assert result['position_rms'] < .05
    assert max(result['requested_moment_rms']) < .03
