"""MAE 5110 Assignment 2 — inverted pendulum walker.

Continuous-time ankle controller (u1 = tau) and the region of attraction it buys.
The discrete-time step controller (u2 = alpha), the Poincare section and the state-action
lookup table are not implemented here: decisions D3-D6 are still open. alpha is held fixed
at its lower bound, which is enough to exercise the RoA event guard.

Sign convention: theta is measured from upward vertical, POSITIVE DOWNHILL, matching
models.inverted_pendulum_walker.visualize, which hardcodes hub = foot + l*[sin, cos],
and matching event_dynamics, which lands at gamma + alpha and resets to gamma - alpha.

# AI-assisted: derivations in assignments/assignment_2/MATH.md sections 1-3 (dynamics,
# impact map, feedback linearization with bounded torque) were worked out with Claude;
# the control law, grid bounds and convergence test were chosen by me in DECISIONS.md.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from models import inverted_pendulum_walker as model

# ------------------------------------------------------
#
# Parameters and tuning constants
#
# ------------------------------------------------------

BRAKING_TORQUE_FRACTION = 0.10  # downhill braking authority
DRIVING_TORQUE_FRACTION = 0.05  # downhill driving authority
ANGLE_OF_ATTACK_BOUNDS = (np.pi / 8, np.pi / 7)  # permitted attack angles

NATURAL_FREQUENCY = np.sqrt(9.81 / 1.0)  # closed loop rate
DAMPING_RATIO = 1.0  # critically damped choice

BALANCE_SETTLING_TIME = 4.0  # balance settling window
RESTING_ANGLE_TOLERANCE = 0.01  # upright angle tolerance
RESTING_ANGULAR_VELOCITY_TOLERANCE = 0.05  # at rest tolerance

ROA_ANGLE_MARGIN = 0  # grid edge margin
ROA_ANGULAR_VELOCITY_BOUNDS = (-2.0, 2.0)  # swept velocity range
ROA_ANGLE_RESOLUTION = 0.005  # roa angle spacing
ROA_ANGULAR_VELOCITY_RESOLUTION = 0.01  # roa velocity spacing

BALANCE_TIMESTEP = 1e-3  # balance sweep timestep
WALK_TIMESTEP = 1e-4  # walking phase timestep
WALK_SIM_TIME = 6.0  # walking time limit
ANIMATION_INITIAL_STATE = np.array([0.0, 3.0])  # animation starting state

SECTION_ANGLE = 0.0  # poincare section angle
SECTION_VELOCITY_MAX = np.sqrt(2 * 9.81 / 1.0)  # froude two ceiling
SECTION_VELOCITY_RESOLUTION_LADDER = (0.64, 0.32, 0.16, 0.08, 0.04, 0.02, 0.01)  # tested velocity spacings
PROBE_SECTION_VELOCITY = 3.0  # resolution probe velocity
ANGLE_OF_ATTACK_COUNT = 9  # action menu size
STEP_MAP_TIMESTEP = 1e-4  # step map timestep
STEP_MAP_MAX_SIM_TIME = 5.0  # step time limit
EXECUTION_RESOLUTION = 0.04  # policy execution spacing

params = {
    "gravity": 9.81,  # downward gravitational acceleration
    "length": 1.0,  # stance leg length
    "mass": 1.0,  # hub point mass
    "incline": 0.06,  # ground slope angle
    "angle_of_attack": np.pi / 8,  # half leg angle
    "ankle_torque": 0.0,  # ankle control input
    "proportional_gain": NATURAL_FREQUENCY**2,  # position feedback gain
    "derivative_gain": 2 * DAMPING_RATIO * NATURAL_FREQUENCY,  # velocity feedback gain
}


# ------------------------------------------------------
#
# Ankle controller
#
# ------------------------------------------------------


def compute_torque_limits(params):
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]

    weight_torque = mass * gravity * length
    return -BRAKING_TORQUE_FRACTION * weight_torque, DRIVING_TORQUE_FRACTION * weight_torque


def compute_balance_limits(params):
    return -np.arcsin(DRIVING_TORQUE_FRACTION), np.arcsin(BRAKING_TORQUE_FRACTION)


def compute_contact_angle_bands(params):
    incline = params["incline"]
    min_angle_of_attack, max_angle_of_attack = ANGLE_OF_ATTACK_BOUNDS

    post_impact_band = (incline - max_angle_of_attack, incline - min_angle_of_attack)
    touchdown_band = (incline + min_angle_of_attack, incline + max_angle_of_attack)
    return post_impact_band, touchdown_band


def compute_ankle_torque(state, params):
    gravity = params["gravity"]
    length = params["length"]
    mass = params["mass"]
    proportional_gain = params["proportional_gain"]
    derivative_gain = params["derivative_gain"]

    angle = state[0]
    angular_velocity = state[1]

    gravity_cancellation = -mass * gravity * length * np.sin(angle)
    virtual_acceleration = -proportional_gain * angle - derivative_gain * angular_velocity
    unsaturated_torque = gravity_cancellation + mass * length**2 * virtual_acceleration

    min_torque, max_torque = compute_torque_limits(params)
    return np.clip(unsaturated_torque, min_torque, max_torque)


def detect_balanced(state):
    angle = state[0]
    angular_velocity = state[1]

    return (np.abs(angle) < RESTING_ANGLE_TOLERANCE) & (
        np.abs(angular_velocity) < RESTING_ANGULAR_VELOCITY_TOLERANCE
    )


# ------------------------------------------------------
#
# Simulation
#
# ------------------------------------------------------


def sweep_roa(params, angle_values, angular_velocity_values, timestep=BALANCE_TIMESTEP, sim_time=BALANCE_SETTLING_TIME):
    angle_grid, angular_velocity_grid = np.meshgrid(
        angle_values, angular_velocity_values, indexing="ij"
    )
    states = np.vstack([angle_grid.ravel(), angular_velocity_grid.ravel()])

    n_timesteps = round(sim_time / timestep) + 1
    for step in range(n_timesteps - 1):
        params["ankle_torque"] = compute_ankle_torque(states, params)
        states = states + timestep * model.dynamics(step * timestep, states, params)

    return detect_balanced(states).reshape(angle_grid.shape)


def detect_roa_entry(state, roa_grid, angle_values, angular_velocity_values):
    angle = state[0]
    angular_velocity = state[1]

    if not (angle_values[0] <= angle <= angle_values[-1]):
        return False
    if not (angular_velocity_values[0] <= angular_velocity <= angular_velocity_values[-1]):
        return False

    angle_index = np.abs(angle_values - angle).argmin()
    angular_velocity_index = np.abs(angular_velocity_values - angular_velocity).argmin()
    return bool(roa_grid[angle_index, angular_velocity_index])


def generate_roa_grid_axes(params, angle_resolution=ROA_ANGLE_RESOLUTION, angular_velocity_resolution=ROA_ANGULAR_VELOCITY_RESOLUTION):
    post_impact_band, touchdown_band = compute_contact_angle_bands(params)
    min_angle = post_impact_band[0] - ROA_ANGLE_MARGIN
    max_angle = touchdown_band[1] + ROA_ANGLE_MARGIN

    angle_values = np.arange(min_angle, max_angle + angle_resolution / 2, angle_resolution)
    angular_velocity_values = np.arange(
        ROA_ANGULAR_VELOCITY_BOUNDS[0],
        ROA_ANGULAR_VELOCITY_BOUNDS[1] + angular_velocity_resolution / 2,
        angular_velocity_resolution,
    )
    return angle_values, angular_velocity_values


def simulate_walk_to_standstill(initial_state, params, roa_grid, angle_values, angular_velocity_values, section_velocities, policy, angle_of_attack_values, timestep=WALK_TIMESTEP, sim_time=WALK_SIM_TIME):
    n_timesteps = round(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    torque_traj = np.zeros(n_timesteps)
    alpha_traj = np.zeros(n_timesteps)
    state_traj[:, 0] = initial_state
    completed_steps = 0
    roa_entry_step = -1

    walk_params = dict(params)
    choice = policy[np.abs(section_velocities - initial_state[1]).argmin()]
    if choice >= 0:
        walk_params["angle_of_attack"] = angle_of_attack_values[choice]
    alpha_traj[0] = walk_params["angle_of_attack"]

    for step, time in enumerate(time_traj[:-1]):
        state = state_traj[:, step]

        if roa_entry_step < 0 and detect_roa_entry(
            state, roa_grid, angle_values, angular_velocity_values
        ):
            roa_entry_step = step

        is_balancing = roa_entry_step >= 0
        walk_params["ankle_torque"] = compute_ankle_torque(state, walk_params) if is_balancing else 0.0
        torque_traj[step] = walk_params["ankle_torque"]

        next_state = state + timestep * model.dynamics(time, state, walk_params)

        if not is_balancing and model.event_guard(state, next_state, walk_params):
            next_state = model.event_dynamics(next_state, walk_params)
            completed_steps += 1

        if not is_balancing and state[0] < SECTION_ANGLE <= next_state[0] and next_state[1] > 0:
            choice = policy[np.abs(section_velocities - next_state[1]).argmin()]
            if choice >= 0:
                walk_params["angle_of_attack"] = angle_of_attack_values[choice]

        state_traj[:, step + 1] = next_state
        alpha_traj[step + 1] = walk_params["angle_of_attack"]

    torque_traj[-1] = torque_traj[-2]
    return time_traj, state_traj, torque_traj, alpha_traj, roa_entry_step, completed_steps


def generate_angle_of_attack_values(count=ANGLE_OF_ATTACK_COUNT):
    return np.linspace(ANGLE_OF_ATTACK_BOUNDS[0], ANGLE_OF_ATTACK_BOUNDS[1], count)


def simulate_step_trajectory(section_velocity, angle_of_attack, params, timestep=STEP_MAP_TIMESTEP):
    step_params = dict(params)
    step_params["angle_of_attack"] = angle_of_attack
    step_params["ankle_torque"] = 0.0

    state = np.array([SECTION_ANGLE, section_velocity])
    max_iterations = round(STEP_MAP_MAX_SIM_TIME / timestep)
    descent = [state]

    for _ in range(max_iterations):
        next_state = state + timestep * model.dynamics(0.0, state, step_params)
        if model.event_guard(state, next_state, step_params):
            state = model.event_dynamics(next_state, step_params)
            break
        state = next_state
        descent.append(state)
    else:
        return np.zeros((2, 0))

    climb = [state]
    for _ in range(max_iterations):
        if state[1] <= 0.0:
            return np.zeros((2, 0))
        next_state = state + timestep * model.dynamics(0.0, state, step_params)
        climb.append(next_state)
        if next_state[0] >= SECTION_ANGLE:
            break
        state = next_state
    else:
        return np.zeros((2, 0))

    gap = np.full((2, 1), np.nan)
    return np.hstack([np.array(descent).T, gap, np.array(climb).T])


def simulate_one_step(section_velocity, angle_of_attack, params, timestep=STEP_MAP_TIMESTEP):
    trajectory = simulate_step_trajectory(section_velocity, angle_of_attack, params, timestep)
    return np.nan if trajectory.size == 0 else trajectory[1, -1]


def build_step_map(section_velocities, angle_of_attack_values, params, timestep=STEP_MAP_TIMESTEP):
    step_map = np.full((section_velocities.size, angle_of_attack_values.size), np.nan)
    for i, section_velocity in enumerate(section_velocities):
        for j, angle_of_attack in enumerate(angle_of_attack_values):
            step_map[i, j] = simulate_one_step(section_velocity, angle_of_attack, params, timestep)
    return step_map


def compute_section_roa_limit(roa_grid, angle_values, angular_velocity_values):
    section_column = roa_grid[np.abs(angle_values - SECTION_ANGLE).argmin()]
    inside = angular_velocity_values[section_column]
    positive = inside[inside > 0]
    return positive.max() if positive.size else 0.0


def compute_steps_to_standstill(section_velocities, step_map, section_roa_limit):
    steps = np.where(section_velocities <= section_roa_limit, 0.0, np.inf)
    policy = np.full(section_velocities.size, -1)

    updated = True
    while updated:
        updated = False
        for i in range(section_velocities.size):
            if steps[i] == 0:
                continue
            for j, next_velocity in enumerate(step_map[i]):
                if np.isnan(next_velocity):
                    continue
                nearest = np.abs(section_velocities - next_velocity).argmin()
                candidate = steps[nearest] + 1
                if candidate < steps[i]:
                    steps[i] = candidate
                    policy[i] = j
                    updated = True
    return steps, policy


def compute_max_steps_before_roa(section_velocities, step_map, section_roa_limit):
    max_steps = np.full(section_velocities.size, np.nan)
    max_steps[section_velocities <= section_roa_limit] = 0.0
    policy = np.full(section_velocities.size, -1)

    for i in range(section_velocities.size):
        if max_steps[i] == 0:
            continue
        for j, next_velocity in enumerate(step_map[i]):
            if np.isnan(next_velocity):
                continue
            nearest = np.abs(section_velocities - next_velocity).argmin()
            if nearest >= i or np.isnan(max_steps[nearest]):
                continue
            candidate = max_steps[nearest] + 1
            if np.isnan(max_steps[i]) or candidate > max_steps[i]:
                max_steps[i] = candidate
                policy[i] = j
    return max_steps, policy


def simulate_policy_walk(initial_section_velocity, params, section_velocities, policy, angle_of_attack_values, section_roa_limit, timestep=STEP_MAP_TIMESTEP, step_limit=20):
    segments = []
    visited = [initial_section_velocity]
    used = []
    velocity = initial_section_velocity

    for _ in range(step_limit):
        if velocity <= section_roa_limit:
            break
        choice = policy[np.abs(section_velocities - velocity).argmin()]
        if choice < 0:
            break
        angle_of_attack = angle_of_attack_values[choice]
        segment = simulate_step_trajectory(velocity, angle_of_attack, params, timestep)
        if segment.size == 0:
            break
        segments.append(segment)
        used.append(angle_of_attack)
        velocity = segment[1, -1]
        visited.append(velocity)

    trajectory = np.hstack(segments) if segments else np.zeros((2, 0))
    return trajectory, np.array(visited), np.array(used)


# ------------------------------------------------------
#
# Plotting
#
# ------------------------------------------------------


def save_figure(filename):
    output = Path("output/assignment_2")
    output.mkdir(parents=True, exist_ok=True)
    plt.savefig(output / filename, dpi=130)


def plot_phase_trajectory(trajectory, section_velocities_visited, params, title):
    post_impact_band, touchdown_band = compute_contact_angle_bands(params)

    plt.figure()
    plt.plot(trajectory[0, :], trajectory[1, :])
    plt.scatter(np.full(section_velocities_visited.size, SECTION_ANGLE),
                section_velocities_visited, color="red", zorder=3, label="Section Crossings")
    plt.axvspan(*post_impact_band, color="b", alpha=0.15, label="Left Bound")
    plt.axvspan(*touchdown_band, color="b", alpha=0.15, label="Right Bound")
    for edge in (*post_impact_band, *touchdown_band):
        plt.axvline(x=edge, color="b", linestyle="--", linewidth=1)
    plt.axvline(x=SECTION_ANGLE, color="k", linestyle=":", label="Poincare Section")
    plt.xlabel("Angle(rad)")
    plt.ylabel("Angular Velocity(rad/sec)")
    plt.title(title)
    plt.grid()
    plt.legend()
    plt.tight_layout()


def plot_steps_to_standstill(section_velocities, steps, section_roa_limit, ax=None):
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 5), layout="constrained")

    reachable = np.isfinite(steps)
    ax.step(section_velocities[reachable], steps[reachable], where="mid", color="#23699b")
    ax.fill_between([0, section_roa_limit], 0, steps[reachable].max() + 0.5,
                    color="#7b5ea7", alpha=0.2, label="already in the ankle RoA")
    if (~reachable).any():
        ax.scatter(section_velocities[~reachable],
                   np.full((~reachable).sum(), steps[reachable].max() + 0.5),
                   marker="x", color="crimson", s=18, label="no legal step sequence")

    ax.set_xlabel(r"initial section velocity $\dot\theta_0$ at $\theta=0$ (rad/s)")
    ax.set_ylabel("footsteps to standstill")
    ax.set_title("Steps to standstill from the vertical Poincare section")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    return ax


def plot_roa(angle_values, angular_velocity_values, roa_grid, params, ax=None):
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 5), layout="constrained")

    ax.pcolormesh(
        angle_values,
        angular_velocity_values,
        roa_grid.T,
        cmap="Greens",
        vmin=0,
        vmax=1.4,
        shading="nearest",
    )

    post_impact_band, touchdown_band = compute_contact_angle_bands(params)
    ax.axvspan(
        *post_impact_band,
        color="#7b5ea7",
        alpha=0.22,
        label=r"post-impact stance, $\gamma-\alpha$, $\alpha\in[\pi/8,\pi/7]$",
    )
    ax.axvspan(
        *touchdown_band,
        color="#df8a25",
        alpha=0.25,
        label=r"touchdown, $\gamma+\alpha$, $\alpha\in[\pi/8,\pi/7]$",
    )
    for edge in (*post_impact_band, *touchdown_band):
        ax.axvline(edge, color="0.35", linewidth=0.8, alpha=0.7)

    min_angle, max_angle = compute_balance_limits(params)
    ax.axvline(min_angle, color="crimson", linestyle="--", label=f"equilibrium limit {min_angle:.4f} rad")
    ax.axvline(max_angle, color="crimson", linestyle="-.", label=f"equilibrium limit {max_angle:+.4f} rad")
    ax.axvline(0.0, color="0.4", linestyle=":", linewidth=1, label="upright")

    ax.set_xlabel("Angle from upward vertical (rad), positive downhill")
    ax.set_ylabel("Angular velocity (rad/s)")
    ax.set_title("Region of attraction of the ankle controller (green = balanced)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=7, framealpha=0.92)
    return ax


# ------------------------------------------------------
#
# Experiments
#
# ------------------------------------------------------


def run_roa():
    angle_values, angular_velocity_values = generate_roa_grid_axes(params)

    roa_grid = sweep_roa(params, angle_values, angular_velocity_values)

    plot_roa(angle_values, angular_velocity_values, roa_grid, params)
    save_figure("roa.png")
    plt.show()


def run_steps():
    angle_values, angular_velocity_values = generate_roa_grid_axes(params)
    roa_grid = sweep_roa(params, angle_values, angular_velocity_values)
    section_roa_limit = compute_section_roa_limit(roa_grid, angle_values, angular_velocity_values)

    angle_of_attack_values = generate_angle_of_attack_values()
    finest = min(SECTION_VELOCITY_RESOLUTION_LADDER)
    fine_velocities = np.arange(0.0, SECTION_VELOCITY_MAX + finest / 2, finest)
    fine_step_map = build_step_map(fine_velocities, angle_of_attack_values, params)

    print(f"D4 resolution check from thetadot_0 = {PROBE_SECTION_VELOCITY:.2f} rad/s:")
    print(f"  {'spacing':>8} {'table':>6} {'walker':>7} {'reaches RoA':>12}")
    results = {}
    for resolution in sorted(SECTION_VELOCITY_RESOLUTION_LADDER, reverse=True):
        stride = round(resolution / finest)
        velocities = fine_velocities[::stride]
        steps, policy = compute_steps_to_standstill(velocities, fine_step_map[::stride], section_roa_limit)
        predicted = steps[np.abs(velocities - PROBE_SECTION_VELOCITY).argmin()]
        _, visited, used = simulate_policy_walk(
            PROBE_SECTION_VELOCITY, params, velocities, policy, angle_of_attack_values, section_roa_limit
        )
        reached = bool(visited[-1] <= section_roa_limit)
        results[resolution] = (velocities, steps, used.size == predicted and reached)
        print(f"  {resolution:>8.2f} {predicted:>6.0f} {used.size:>7} {reached!s:>12}")

    ladder = sorted(SECTION_VELOCITY_RESOLUTION_LADDER, reverse=True)
    chosen = next((r for i, r in enumerate(ladder) if all(results[f][2] for f in ladder[i:])), finest)

    velocities, steps = results[chosen][:2]
    plot_steps_to_standstill(velocities, steps, section_roa_limit)
    save_figure("steps_to_standstill.png")
    plt.show()


def run_trajectory():
    angle_values, angular_velocity_values = generate_roa_grid_axes(params)
    roa_grid = sweep_roa(params, angle_values, angular_velocity_values)
    section_roa_limit = compute_section_roa_limit(roa_grid, angle_values, angular_velocity_values)

    angle_of_attack_values = generate_angle_of_attack_values()
    velocities = np.arange(0.0, SECTION_VELOCITY_MAX + EXECUTION_RESOLUTION / 2, EXECUTION_RESOLUTION)
    step_map = build_step_map(velocities, angle_of_attack_values, params)

    _, fewest_policy = compute_steps_to_standstill(velocities, step_map, section_roa_limit)
    _, most_policy = compute_max_steps_before_roa(velocities, step_map, section_roa_limit)

    start = velocities[np.abs(velocities - PROBE_SECTION_VELOCITY).argmin()]

    for name, policy in (("Fewest Steps", fewest_policy), ("Most Steps", most_policy)):
        trajectory, visited, used = simulate_policy_walk(
            start, params, velocities, policy, angle_of_attack_values, section_roa_limit
        )
        plot_phase_trajectory(trajectory, visited, params,
                              f"{name} to Standstill ({used.size} steps)")
        save_figure(f"trajectory_{name.split()[0].lower()}.png")

    plt.show()


def run_animation():
    angle_values, angular_velocity_values = generate_roa_grid_axes(params)
    roa_grid = sweep_roa(params, angle_values, angular_velocity_values)
    section_roa_limit = compute_section_roa_limit(roa_grid, angle_values, angular_velocity_values)
    angle_of_attack_values = generate_angle_of_attack_values()
    velocities = np.arange(0.0, SECTION_VELOCITY_MAX + EXECUTION_RESOLUTION / 2, EXECUTION_RESOLUTION)
    step_map = build_step_map(velocities, angle_of_attack_values, params)
    _, policy = compute_steps_to_standstill(velocities, step_map, section_roa_limit)

    time_traj, state_traj, torque_traj, alpha_traj, roa_entry_step, _ = (
        simulate_walk_to_standstill(
            ANIMATION_INITIAL_STATE, params, roa_grid, angle_values, angular_velocity_values,
            velocities, policy, angle_of_attack_values,
        )
    )

    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")

    def draw_frame(index):
        is_balancing = 0 <= roa_entry_step <= index
        frame_params = {**params, "ankle_torque": torque_traj[index], "angle_of_attack": alpha_traj[index]}
        model.visualize(state_traj[:, index], frame_params, ax=ax, show_swing=not is_balancing)
        label = "balancing" if is_balancing else "walking"
        ax.set_title(f"t = {time_traj[index]:.2f} s  ({label})")

    fps = 25
    frame_stride = round(1 / (fps * WALK_TIMESTEP))
    frame_indices = list(range(0, time_traj.size, frame_stride))
    if frame_indices[-1] != time_traj.size - 1:
        frame_indices.append(time_traj.size - 1)

    animation = FuncAnimation(fig, draw_frame, frames=frame_indices, interval=1000 / fps, repeat=False)
    output = Path("output/assignment_2")
    output.mkdir(parents=True, exist_ok=True)
    animation.save(output / "walker.gif", writer=PillowWriter(fps=fps))
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="MAE5110 Assignment 2 experiments.")
    parser.add_argument(
        "--experiment",
        choices=["roa", "steps", "trajectory", "animation"],
        default="animation",
    )
    args = parser.parse_args()

    experiments = {
        "roa": run_roa,
        "steps": run_steps,
        "trajectory": run_trajectory,
        "animation": run_animation,
    }
    experiments[args.experiment]()


if __name__ == "__main__":
    main()
