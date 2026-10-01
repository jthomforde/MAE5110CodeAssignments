import numpy as np

from integrators import rk4
from models import pendulum


def test_energy_conservation():
    params = pendulum.generate_params()
    params["damping_coeff"] = 0.0
    params["torque"] = 0.0

    state = np.array([0.3, 0.5])
    timestep = 1e-3

    initial_energy = sum(pendulum.calculate_energy(state, params))

    for step in range(100):
        state = rk4(pendulum.dynamics, step * timestep, state, timestep, params)

    final_energy = sum(pendulum.calculate_energy(state, params))

    assert np.isclose(final_energy, initial_energy, rtol=1e-6)


def test_damping():
    params = pendulum.generate_params()
    params["gravity"] = 0.0
    params["torque"] = 0.0
    params["damping_coeff"] = 0.5

    state = np.array([0.0, 2.0])
    derivative = pendulum.dynamics(0.0, state, params)

    expected_acceleration = -params["damping_coeff"] * state[1] / (
        params["mass"] * params["length"] ** 2
    )

    assert np.isclose(derivative[1], expected_acceleration)


def test_torque():
    params = pendulum.generate_params()
    params["gravity"] = 0.0
    params["damping_coeff"] = 0.0
    params["torque"] = 2.0

    state = np.array([0.0, 0.0])
    derivative = pendulum.dynamics(0.0, state, params)

    expected_acceleration = params["torque"] / (
        params["mass"] * params["length"] ** 2
    )

    assert np.isclose(derivative[1], expected_acceleration)