import numpy as np

def integrate(dynamics, timestep, sim_time, initial_state, params, detect_impact, apply_impact):
    
    
    n_timesteps = int(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = initial_state
    

    # simulation loop
    for step, t in enumerate(time_traj[:-1]):

        state_traj[:, step + 1] = state_traj[:, step] + timestep * dynamics(t, state_traj[:, step], params)

        #overwrite the state + 1 if an impact is detected
        if(detect_impact(state_traj[:, step + 1], params)):
            state_traj[:, step + 1] = apply_impact(state_traj[:,step + 1],params)
            # print("impact", state_traj[:,step + 1])

    return time_traj, state_traj

