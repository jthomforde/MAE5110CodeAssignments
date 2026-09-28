import numpy as np
import matplotlib.pyplot as plt
import timeit

from models import pendulum, bouncing_ball
from integrators import rk4, explicit_euler


# Basic simulation of the pendulum

params = {
    "gravity": 9.81,  # gravity m/s^2)
    "length": 1,  # rod length (m)
    "mass": 0.2,  # point mass at end of rod (kg)
    "damping_coeff": 0.0,  # damping coefficient (kg*m^2/s)
    "stiffness": 10000  # stiffness of the ground (spring behavior) (N/m)
    "torque": 0.0,  # torque (Nm)
}


# some set-up
initial_state = np.array([1, 0.0])



#simulation sweep
# timestep = 0.0016

# timestep_modifier = .00001
# simulation_stable = True
# while(simulation_stable):

#     timestep += timestep_modifier
#     sim_time = 5.0

#     time_traj, state_traj = rk4.integrate(model.dynamics, timestep, sim_time, initial_state, params)
    
#     kinetic_energy, potential_energy = model.calculate_energy(state_traj, params)
#     print(f"ran sweep at timestep: {timestep}")
#     print(kinetic_energy[0] + potential_energy[0])
#     print(kinetic_energy[-1] + potential_energy[-1])
#     if abs((kinetic_energy[0] + potential_energy[0]) - (kinetic_energy[-1] + potential_energy[-1])) > 0.1:
#         simulation_stable = False
#         print(timestep)
#         print(timestep_modifier)
#         print(kinetic_energy[0] + potential_energy[0])
#         print(kinetic_energy[-1] + potential_energy[-1])



#Timeit comparison
#rk4_timestep = 0.27052
# explicit_euler_timestep = 0.00172
# common_timestep = 0.00001
# sim_time = 5.0


# print("RK4 time at max timestep:", timeit.timeit(lambda: rk4.integrate(model.dynamics, rk4_timestep, sim_time, initial_state, params), number=1))
# print("Explicit Euler time at max timestep:", timeit.timeit(lambda: explicit_euler.integrate(model.dynamics, explicit_euler_timestep, sim_time, initial_state, params), number=1))

# print("RK4 time at common timestep:", timeit.timeit(lambda: rk4.integrate(model.dynamics, common_timestep, sim_time, initial_state, params), number=1))
# print("Explicit Euler time at common timestep:", timeit.timeit(lambda: explicit_euler.integrate(model.dynamics, common_timestep, sim_time, initial_state, params), number=1))


# sanity check the energies: since there is no actuation, and no damping, total energy should stay
# constant. If we turn on the damping coefficient, it should slowly bleed out energy until it comes to
# a stand-still.

common_timestep = 0.00001
sim_time = 5.0

time_traj, state_traj = rk4.integrate(bouncing_ball.dynamics, common_timestep, sim_time, initial_state, params)

kinetic_energy, potential_energy = bouncing_ball.calculate_energy(state_traj, params)

plt.figure()
plt.plot(time_traj, potential_energy, label="Potential energy")
plt.plot(time_traj, kinetic_energy, label="Kinetic energy")
plt.plot(time_traj, potential_energy + kinetic_energy, label="Total energy")
plt.xlabel("Time (s)")
plt.ylabel("Energy (J)")
plt.title("Pendulum energy")
plt.legend()
plt.tight_layout()
plt.show()



# TODO: make a phase portrait plot

# plt.figure()
# plt.plot(state_traj[1], state_traj[0], label="Phase Portrait")
# plt.xlabel("Velocity (m/s)")
# plt.ylabel("Height (m)")
# plt.title("Pendulum Phase Portrait")
# plt.legend()
# plt.tight_layout()
# plt.show()

