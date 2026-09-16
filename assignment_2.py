from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from models import inverted_pendulum_walker as model
# Fixed controls for this visualization example.
params = {
    "gravity": 9.81,  # m/s^2
    "length": 1.0,  # m
    "mass": 1.0,  # kg
    "incline": 0.06,  # rad
    "angle_of_attack": np.pi / 8,  # rad
    "ankle_torque": 0.0,  # N m
}


def compute_torque_limits(params):
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]
    ankle_torque = params["ankle_torque"]

    return -gravity * mass * length * 0.1, gravity * mass * length * 0.05

def calculate_torque(params, state, damping):
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]
    ankle_torque = params["ankle_torque"]

    angle = state[0]
    angular_velocity = state[1]

    min_torque, max_torque = compute_torque_limits(params)
    artificial_attraction_torque = -2 * mass * gravity * length * np.sin(angle)
    unbounded_torque = artificial_attraction_torque - damping * angular_velocity

    return np.clip(unbounded_torque, min_torque, max_torque)

#------------------------------------
# Graphing Sweeps and Visualizer
#------------------------------------

initial_state = np.array([0, 1])
timestep = 1e-4
sim_time = 6.0
desired_number_of_steps = 5
#set up
n_timesteps = round(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep
state_traj = np.zeros((2, n_timesteps))
state_traj[:, 0] = initial_state
completed_steps = 0




def inverted_pendulum_walker_integrator(initial_state, params, time_traj, state_traj, completed_steps):
    # Simulation loop. Replace this Euler step with your own integrator as needed.
    state = initial_state
    for step, t in enumerate(time_traj[:-1]):

        params["ankle_torque"] = calculate_torque(params, state, 0.5)
        state = state_traj[:, step]
        next_state = state + timestep * model.dynamics(t, state, params)

        if model.event_guard(state, next_state, params):
            next_state = model.event_dynamics(next_state, params)
            completed_steps += 1

        state_traj[:, step + 1] = next_state
        if completed_steps == desired_number_of_steps:
            break

    time_traj = time_traj[: step + 2]
    state_traj = state_traj[:, : step + 2]
    return time_traj, state_traj



def torque_roa_sweep(params, ):
    LEFT, RIGHT = params["incline"] - params["angle_of_attack"], params["incline"] + params["angle_of_attack"]
    theta_grid_steps = np.arange(LEFT, RIGHT, 0.1)
    thetadot_grid_steps = np.arange(-10, 10, .5)

    not_torque_stability_converged = []
    torque_stability_converged = []

    initial_state = np.array([0.1, 1])
    timestep = 1e-4
    sim_time = 3.0
    #set up
    n_timesteps = round(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = initial_state
    completed_steps = 0

    for theta in theta_grid_steps:
            for thetadot in thetadot_grid_steps:
                initial_state = np.array([theta, thetadot])
                time_traj, state_traj = inverted_pendulum_walker_integrator(initial_state, params, time_traj, state_traj, completed_steps)
            
                if(state_traj[1,-1]  < 0.1 & state_traj[0,-1] < 0.1 & state_traj[0,-1] > 0.05):
                    torque_stability_converged.append(initial_state)
                else:
                    not_torque_stability_converged.append(initial_state)


    torque_stability_converged = np.array(torque_stability_converged)
    not_torque_stability_converged     = np.array(not_torque_stability_converged)

    # Plotting the limit cycle line - AI assisted
    # gravity, L = params["gravity"], params["length"]
    # a, ramp_angle = params["alpha"], params["ramp_angle"]
    # LEFT, RIGHT = ramp_angle - a, ramp_angle + a

    # # The wheel rolls toward whichever guard has lower potential energy
    # # (PE = m*g*L*cos(theta)), lands there, and is reset to the other one.
    # if np.cos(LEFT) < np.cos(RIGHT):
    #     start, land, sign = RIGHT, LEFT, -1.0     # rolls toward -theta
    # else:
    #     start, land, sign = LEFT, RIGHT, +1.0     # rolls toward +theta

    # c = np.cos(2 * a)
    # D = (2 * gravity / L) * (np.cos(start) - np.cos(land))
    # omega_star = np.sqrt(c**2 * D / (1 - c**2))   # post-impact speed on the cycle

    # theta_cycle = np.linspace(start, land, 400)
    # thetadot_cycle = sign * np.sqrt(omega_star**2 + (2 * gravity / L) * (np.cos(start) - np.cos(theta_cycle)))


    plt.figure()
    plt.scatter(torque_stability_converged[:, 0], torque_stability_converged[:, 1], label="Limit Cycle Converged", color='green')
    if not_torque_stability_converged.size > 0:
        plt.scatter(not_torque_stability_converged[:, 0], not_torque_stability_converged[:, 1], label="Stopped Converged", color='red')
    plt.xlabel("Angle(rad)")
    plt.ylabel("Angular Momentum(rad/sec)")
    plt.axvline(x=RIGHT, color='b', linestyle='--', label="Right Bound")
    plt.axvline(x=LEFT, color='b', linestyle='--', label="Left Bound")
    # plt.axhline(y=0, color='red', linestyle='--', label="Came to rest(Attractor)")
    # plt.plot(theta_cycle, thetadot_cycle, 'k', lw=2, label="Limit cycle(Attractor)", color='green')
    plt.title("Rimless Wheel Phase Portrait")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()



state = initial_state
for step, t in enumerate(time_traj[:-1]):

    params["ankle_torque"] = calculate_torque(params, state, 0.5)
    state = state_traj[:, step]
    next_state = state + timestep * model.dynamics(t, state, params)

    if model.event_guard(state, next_state, params):
        next_state = model.event_dynamics(next_state, params)
        completed_steps += 1

    state_traj[:, step + 1] = next_state
    if completed_steps == desired_number_of_steps:
        break

time_traj = time_traj[: step + 2]
state_traj = state_traj[:, : step + 2]

fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")



def draw_frame(index):
    # The massless swing leg is repositioned instantaneously at each impact.
    model.visualize(state_traj[:, index], params, ax=ax)
    ax.set_title(f"t = {time_traj[index]:.2f} s")


# Simulate at a small timestep, but render only 25 frames per second.
fps = 25
frame_stride = round(1 / (fps * timestep))
frame_indices = list(range(0, time_traj.size, frame_stride))
if frame_indices[-1] != time_traj.size - 1:
    frame_indices.append(time_traj.size - 1)

animation = FuncAnimation(
    fig, draw_frame, frames=frame_indices, interval=1000 / fps, repeat=False
)
output = Path("output/assignment_2")
output.mkdir(parents=True, exist_ok=True)
animation.save(output / "walker.gif", writer=PillowWriter(fps=fps))

# To save an MP4 instead, install FFmpeg and use:
# animation.save(output / "walker.mp4", writer="ffmpeg", fps=fps)
print(f"Saved {output / 'walker.gif'} ({completed_steps} footstrikes).")
plt.show()

#torque_roa_sweep(params)