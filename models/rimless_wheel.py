import numpy as np


def dynamics(t, state, params):
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]
    ramp_angle = params["ramp_angle"]
    number_of_spokes = params["number_of_spokes"]
    alpha = params["alpha"]


    angle = state[0]
    angular_velocity = state[1]


    angular_acceleration = (
        mass * gravity * length * np.sin(angle)
        - 0 * angular_velocity  # <-- DAMPING TERM
    ) / (mass * length**2)

    state_derivative = np.array([angular_velocity, angular_acceleration])
    return state_derivative

def detect_impact(state,params):
    ramp_angle = params["ramp_angle"]
    alpha = params["alpha"]

    angle = state[0]
    angular_velocity = state[1]

    if(angular_velocity > 0):
        return angle >= (ramp_angle + alpha) #uphill
    return angle <= (ramp_angle - alpha)  #downhill

def apply_impact(state,params):
    ramp_angle = params["ramp_angle"]
    alpha = params["alpha"]

    angle = state[0]
    angular_velocity = state[1]

    if (angular_velocity > 0): 
        new_angle = -alpha + ramp_angle
        # print(new_angle, "backwards")
        # print(alpha, ramp_angle)
        new_angular_velocity = angular_velocity * np.cos(2 * alpha)
    else:
        new_angle = ramp_angle + alpha
        # print(new_angle, "forwards")
        # print(alpha, ramp_angle)
        new_angular_velocity = angular_velocity * np.cos(2 * alpha)
    return np.array([new_angle, new_angular_velocity])


def generate_params():
    params = {
        "gravity": 9.81,  # gravity m/s^2)
        "length": 0.5,  # rod length (m)
        "mass": 1,  # point mass at end of rod (kg)
        "ramp_angle": np.pi / 6,  # ramp angle (rad)
        "number_of_spokes": 5,
        "alpha": np.pi / 5   # spoke angle (rad)

    }
    return params

# def can_pass_vertical(state, params):
#     """True if the hub can carry over the vertical (theta = 0) from here.

#     theta = 0 is the potential-energy maximum, and it lies inside the swing
#     arc whenever alpha > ramp_angle. If the wheel cannot reach it, it is
#     trapped rocking between two spokes and will come to rest. Mass cancels.
#     """
#     gravity = params["gravity"]
#     length = params["length"]

#     angle = state[0]
#     angular_velocity = state[1]

#     energy = 0.5 * (length * angular_velocity) ** 2 + gravity * length * np.cos(angle)
#     return energy > gravity * length



def calculate_energy(state, params):
    """Compute energies for a state ``(2,)`` or trajectory ``(2, N)``."""
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]

    angle = state[0]  # indexes entire row "vectorized" if state is (2, N)
    angular_velocity = state[1]

    kinetic_energy = 0.5 * mass * (length * angular_velocity) ** 2
    potential_energy = mass * gravity * length * np.cos(angle)
    return kinetic_energy, potential_energy