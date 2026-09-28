import numpy as np
import matplotlib.pyplot as plt
import timeit
import argparse

from models import rimless_wheel
from integrators import rimless_wheel_integrator


# Basic simulation of the rimless_wheel

params = {
    "gravity": 9.81,  # gravity m/s^2)
    "length": 0.5,  # rod length (m)
    "mass": 0.2,  # point mass at end of rod (kg)
    "ramp_angle": -np.pi / 9,  # ramp angle (rad)
    "number_of_spokes": 6,
    "alpha": np.pi / 6   # spoke angle (rad)
}


# Set Up

initial_state = np.array([-0.1, 0])
common_timestep = 0.0001
sim_time = 5.0
LEFT, RIGHT = params["ramp_angle"] - params["alpha"], params["ramp_angle"] + params["alpha"]

# ------------------------------------------------------
# 
# Model Simulation
# 
# ------------------------------------------------------

def simulate_rimless_wheel(initial_state, params, common_timestep, sim_time):

    time_traj, state_traj = rimless_wheel_integrator.integrate(rimless_wheel.dynamics, common_timestep, sim_time, initial_state, params, rimless_wheel.detect_impact, rimless_wheel.apply_impact)


    plt.figure()
    plt.plot(state_traj[0, :], state_traj[1, :])
    plt.xlabel("Angle(rad)")
    plt.ylabel("Angular Velocity(rad/sec)")
    plt.title("Rimless Wheel Phase Portrait")
    plt.axvline(x=params["ramp_angle"] + params["alpha"], color='b', linestyle='--', label="Right Bound")
    plt.axvline(x=params["ramp_angle"] - params["alpha"], color='b', linestyle='--', label="Left Bound")
    plt.show()


# ------------------------------------------------------
# 
# ROA Sweep Functions
# 
# ------------------------------------------------------



def spoke_number_ROA_sweep(params, common_timestep, sim_time):
    
    thetadot_grid_steps = np.arange(-10, 10, 1)
    num_spokes_steps = np.arange(6, 13, 1)

    limit_cycle_converged = []
    stopped_converged = []
    percent_limit_cycle_converged = []

    # params["ramp_angle"] = -np.pi/5
    for num_spokes in num_spokes_steps:

        params["number_of_spokes"] = num_spokes
        params["alpha"] = np.pi / num_spokes
        LEFT, RIGHT = params["ramp_angle"] - params["alpha"], params["ramp_angle"] + params["alpha"]
        theta_grid_steps = np.arange(LEFT, RIGHT, 0.2)
        for theta in theta_grid_steps:
            
            for thetadot in thetadot_grid_steps:
                initial_state = np.array([theta, thetadot])
                time_traj, state_traj = rimless_wheel_integrator.integrate(rimless_wheel.dynamics, common_timestep, sim_time, initial_state, params, rimless_wheel.detect_impact, rimless_wheel.apply_impact)
            
                if(state_traj[1,-1]  < -0.1):
                    limit_cycle_converged.append(initial_state)
                else:
                    stopped_converged.append(initial_state)

        percent_limit_cycle_converged.append(len(limit_cycle_converged) / (len(limit_cycle_converged) + len(stopped_converged)) * 100)
        limit_cycle_converged = []
        stopped_converged = []
    
    plt.figure()
    plt.plot(num_spokes_steps, percent_limit_cycle_converged, label="Percent Limit Cycle Converged")
    plt.xlabel("Number of Spokes")
    plt.ylabel("Percent Converged")
    plt.title("Effect of Number of Spokes on Limit Cycle Convergence")
    plt.legend()
    plt.grid()
    plt.show()



def ramp_angle_ROA_sweep(params, common_timestep, sim_time):
    LEFT, RIGHT = params["ramp_angle"] - params["alpha"], params["ramp_angle"] + params["alpha"]
    
    thetadot_grid_steps = np.arange(-10, 10, 1)
    ramp_angle_steps = np.arange(-5 *np.pi/180, -45 * np.pi/180, -5 *np.pi/180)

    limit_cycle_converged = []
    stopped_converged = []
    percent_limit_cycle_converged = []

    # params["ramp_angle"] = -np.pi/5
    for ramp_angle in ramp_angle_steps:
        params["ramp_angle"] = ramp_angle
        LEFT, RIGHT = params["ramp_angle"] - params["alpha"], params["ramp_angle"] + params["alpha"]
        theta_grid_steps = np.arange(LEFT, RIGHT, 0.2)
        for theta in theta_grid_steps:
            for thetadot in thetadot_grid_steps:
                initial_state = np.array([theta, thetadot])
                time_traj, state_traj = rimless_wheel_integrator.integrate(rimless_wheel.dynamics, common_timestep, sim_time, initial_state, params, rimless_wheel.detect_impact, rimless_wheel.apply_impact)
            
                if(state_traj[1,-1]  < -0.1):
                    limit_cycle_converged.append(initial_state)
                else:
                    stopped_converged.append(initial_state)

        percent_limit_cycle_converged.append(len(limit_cycle_converged) / (len(limit_cycle_converged) + len(stopped_converged)) * 100)
        limit_cycle_converged = []
        stopped_converged = []




    limit_cycle_converged = np.array(limit_cycle_converged)
    stopped_converged     = np.array(stopped_converged)




    plt.figure()
    plt.plot(ramp_angle_steps, percent_limit_cycle_converged, label="Percent Limit Cycle Converged")
    plt.xlabel("Ramp Angle (rad)")
    plt.ylabel("Percent Converged")
    plt.title("Effect of Ramp Angle on Limit Cycle Convergence")
    plt.legend()
    plt.grid()
    plt.show()



def ROA_stability_sweep(params, common_timestep, sim_time):
    LEFT, RIGHT = params["ramp_angle"] - params["alpha"], params["ramp_angle"] + params["alpha"]
    theta_grid_steps = np.arange(LEFT, RIGHT, 0.1)
    thetadot_grid_steps = np.arange(-10, 10, .5)

    limit_cycle_converged = []
    stopped_converged = []

    for theta in theta_grid_steps:
            for thetadot in thetadot_grid_steps:
                initial_state = np.array([theta, thetadot])
                time_traj, state_traj = rimless_wheel_integrator.integrate(rimless_wheel.dynamics, common_timestep, sim_time, initial_state, params, rimless_wheel.detect_impact, rimless_wheel.apply_impact)
            
                if(state_traj[1,-1]  < -0.1):
                    limit_cycle_converged.append(initial_state)
                else:
                    stopped_converged.append(initial_state)


    limit_cycle_converged = np.array(limit_cycle_converged)
    stopped_converged     = np.array(stopped_converged)

    # Plotting the limit cycle line - AI assisted
    gravity, L = params["gravity"], params["length"]
    a, ramp_angle = params["alpha"], params["ramp_angle"]
    LEFT, RIGHT = ramp_angle - a, ramp_angle + a

    # The wheel rolls toward whichever guard has lower potential energy
    # (PE = m*g*L*cos(theta)), lands there, and is reset to the other one.
    if np.cos(LEFT) < np.cos(RIGHT):
        start, land, sign = RIGHT, LEFT, -1.0     # rolls toward -theta
    else:
        start, land, sign = LEFT, RIGHT, +1.0     # rolls toward +theta

    c = np.cos(2 * a)
    D = (2 * gravity / L) * (np.cos(start) - np.cos(land))
    omega_star = np.sqrt(c**2 * D / (1 - c**2))   # post-impact speed on the cycle

    theta_cycle = np.linspace(start, land, 400)
    thetadot_cycle = sign * np.sqrt(omega_star**2 + (2 * gravity / L) * (np.cos(start) - np.cos(theta_cycle)))


    plt.figure()
    plt.scatter(limit_cycle_converged[:, 0], limit_cycle_converged[:, 1], label="Limit Cycle Converged", color='green')
    if stopped_converged.size > 0:
        plt.scatter(stopped_converged[:, 0], stopped_converged[:, 1], label="Stopped Converged", color='red')
    plt.xlabel("Angle(rad)")
    plt.ylabel("Angular Momentum(rad/sec)")
    plt.axvline(x=params["ramp_angle"] + params["alpha"], color='b', linestyle='--', label="Right Bound")
    plt.axvline(x=params["ramp_angle"] - params["alpha"], color='b', linestyle='--', label="Left Bound")
    plt.axhline(y=0, color='red', linestyle='--', label="Came to rest(Attractor)")
    plt.plot(theta_cycle, thetadot_cycle, 'k', lw=2, label="Limit cycle(Attractor)", color='green')
    plt.title("Rimless Wheel Phase Portrait")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()



# ------------------------------------------------------
# 
# Return Map and Floquet Multiplier Supporting Functions
# 
# ------------------------------------------------------



def contact_geometry(params):
    left  = params["ramp_angle"] - params["alpha"]
    right = params["ramp_angle"] + params["alpha"]
    if np.cos(left) < np.cos(right):
        return right, left, -1.0
    return left, right, +1.0

def simulate_to_next_impact(angular_velocity_before_step):
    start, land, sign = contact_geometry(params) 
    state = np.array([start, sign * abs(angular_velocity_before_step)])
    t = 0.0
    while t < sim_time:
        state = state + common_timestep * rimless_wheel.dynamics(t, state, params)
        t += common_timestep
        if sign * state[0] < sign * start and sign * state[1] < 0:
            return np.nan
        if rimless_wheel.detect_impact(state, params):
            return abs(rimless_wheel.apply_impact(state, params)[1])
    return np.nan


def compute_return_map(launch_speeds):
    return np.array([simulate_to_next_impact(w) for w in launch_speeds])


def find_fixed_point(launch_speeds, next_speeds):
    valid = np.isfinite(next_speeds)
    w, p = launch_speeds[valid], next_speeds[valid]
    difference = p - w
    crossings = np.where(difference[:-1] * difference[1:] <= 0)[0]
    if len(crossings) == 0:
        return np.nan
    i = crossings[0]
    return w[i] - difference[i] * (w[i + 1] - w[i]) / (difference[i + 1] - difference[i])


def estimate_floquet_multiplier(fixed_point, perturbation):
    faster = simulate_to_next_impact(fixed_point + perturbation)
    slower = simulate_to_next_impact(fixed_point - perturbation)
    return (faster - slower) / (2 * perturbation)



launch_speeds = np.linspace(0.5, 9.0, 90)
next_speeds = compute_return_map(launch_speeds)
fixed_point = find_fixed_point(launch_speeds, next_speeds)


# ------------------------------------------------------
# 
# Return Map and Floquet Multiplier Generation Functions
# 
# ------------------------------------------------------

def generate_return_map(params):
    start, land, sign = contact_geometry(params)
    angular_velocity_at_impact_multiplier = np.cos(2 * params["alpha"])
    energy_gained_over_step = (2 * params["gravity"] / params["length"]) * (np.cos(start) - np.cos(land))
    omega_star = np.sqrt(angular_velocity_at_impact_multiplier**2 * energy_gained_over_step / (1 - angular_velocity_at_impact_multiplier**2))   # post-impact speed on the cycle


    # ---- step-to-step return map ----
    w = np.linspace(0.1, 3.5, 300)

    plt.figure()
    plt.plot(w, angular_velocity_at_impact_multiplier * np.sqrt(w**2 + energy_gained_over_step), lw=2, label=r"Return map $P(\omega)$")
    plt.plot(w, w, '--', label=r"Identity $\omega_{n+1} = \omega_n$")
    plt.plot(omega_star, omega_star, 'o', ms=9, label="Fixed point")
    plt.xlabel(r"$\omega_n^+$ [rad/s]")
    plt.ylabel(r"$\omega_{n+1}^+$ [rad/s]")
    plt.title("Rimless Wheel Step-to-Step Return Map")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()



def floquet_multiplier_vs_ramp_angle(ramp_angles, perturbation):
    multipliers = []
    for ramp_angle in ramp_angles:
        params["number_of_spokes"] = 6
        params["alpha"] = np.pi / 6
        params["ramp_angle"] = ramp_angle
        next_speeds = compute_return_map(launch_speeds)
        fixed_point = find_fixed_point(launch_speeds, next_speeds)
        multiplier = estimate_floquet_multiplier(fixed_point, perturbation)
        multipliers.append(multiplier)
    
    plt.figure()
    plt.plot(ramp_angles, multipliers, label="Floquet Multiplier")
    plt.xlabel("Ramp Angle (rad)")
    plt.ylabel("Floquet Multiplier")
    plt.title("Floquet Multiplier vs Ramp Angle")
    plt.legend()
    plt.grid()
    plt.show()




def floquet_multiplier_vs_spokes(spoke_counts, perturbation):
    multipliers = []
    for spoke_count in spoke_counts:
        params["ramp_angle"] = -np.pi / 9
        params["number_of_spokes"] = spoke_count
        params["alpha"] = np.pi / spoke_count
        next_speeds = compute_return_map(launch_speeds)
        fixed_point = find_fixed_point(launch_speeds, next_speeds)
        multiplier = estimate_floquet_multiplier(fixed_point, perturbation)
        multipliers.append(multiplier)
    
    plt.figure()
    plt.plot(spoke_counts, multipliers, label="Floquet Multiplier")
    plt.axhline(y=1.0, color='r', linestyle='--', label="Stability Threshold")
    plt.xlabel("Number of Spokes")
    plt.ylabel("Floquet Multiplier")
    plt.title("Floquet Multiplier vs Number of Spokes")
    plt.legend()
    plt.grid()
    plt.show()






# ------------------------------------------------------
#
# Running Function 
# 
# ------------------------------------------------------

def run_initial_example():
    simulate_rimless_wheel(initial_state, params, common_timestep, sim_time)


def run_return_map():
    generate_return_map(params)


def run_roa():
    ROA_stability_sweep(params, common_timestep, sim_time)


def run_roa_ramp_sweep():
    ramp_angle_ROA_sweep(params, common_timestep, sim_time)


def run_roa_spoke_sweep():
    spoke_number_ROA_sweep(params, common_timestep, sim_time)


def run_floquet_ramp_sweep():
    floquet_multiplier_vs_ramp_angle(
        np.arange(-5 * np.pi / 180, -45 * np.pi / 180, -5 * np.pi / 180),
        perturbation=0.1,
    )


def run_floquet_spoke_sweep():
    floquet_multiplier_vs_spokes(np.arange(6, 12), perturbation=0.1)


# ------------------------------------------------------
#
# Main Function 
# 
# ------------------------------------------------------


def main():
    
    parser = argparse.ArgumentParser(
        description="MAE5110 Assignment 1 experiments."
    )

    parser.add_argument(
        "--experiment",
        choices=[
            "initial",
            "roa",
            "return_map",
            "roa_ramp_sweep",
            "roa_spoke_sweep",
            "floquet_ramp_sweep",
            "floquet_spoke_sweep",
        ],
        default="initial",
    )

    args = parser.parse_args()

    experiments = {
        "initial": run_initial_example,
        "roa": run_roa,
        "return_map": run_return_map,
        "roa_ramp_sweep": run_roa_ramp_sweep,
        "roa_spoke_sweep": run_roa_spoke_sweep,
        "floquet_ramp_sweep": run_floquet_ramp_sweep,
        "floquet_spoke_sweep": run_floquet_spoke_sweep,
    }

    experiments[args.experiment]()


if __name__ == "__main__":
    main()





