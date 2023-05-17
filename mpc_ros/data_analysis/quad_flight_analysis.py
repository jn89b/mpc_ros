import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os 
import ast
import numpy as np
from array import array
from matplotlib.animation import FuncAnimation
import pickle as pkl

plt.close('all')

def get_command_ref(df:dict):
    x_traj = df['trajectory_y']

    y_traj = df['trajectory_x']
    z_traj = df['trajectory_z']
    # phi_traj = df['roll_trajectory']
    # theta_traj = df['pitch_trajectory']
    # psi_traj = df['yaw_trajectory']
    idx_history = df['idx']

    x_cmd = []
    y_cmd = []
    z_cmd = []
    # phi_cmd = []xx`
    # theta_cmd = []
    # psi_cmd = []

    for x,y,z,idx in zip(x_traj, 
                        y_traj, 
                        z_traj, 
                        idx_history):
        
        x_cmd.append(x[idx-3])
        y_cmd.append(y[idx-3])
        z_cmd.append(z[idx-3])
        # phi_cmd.append(phi[0][idx])
        # theta_cmd.append(theta[0][idx])
        # psi_cmd.append(psi[0][idx])
    
    return x_cmd, y_cmd, z_cmd


#load pickle file
with open('shes_the_one_sim.pkl', 'rb') as f:
    df = pkl.load(f)


#quad data is ned need to convert to enu
start_idx = 100
idx_end = 5800
#parse out df from start to idx_end
x_history = df['y_history'][start_idx:idx_end]
y_history =  df['x_history'][start_idx:idx_end]
z_history = df['z_history'][start_idx:idx_end]
roll_history = df['roll_history'][start_idx:idx_end]
pitch_history = df['pitch_history'][start_idx:idx_end]
yaw_history = df['yaw_history'][start_idx:idx_end]
# obstacles = df['obstacles']
x_trajectory = df['trajectory_y'][:idx_end]
y_trajectory = df['trajectory_x'][:idx_end]
z_trajectory = df['trajectory_z'][:idx_end]
x_trajectory = [list(x) for x in x_trajectory]
y_trajectory = [list(y) for y in y_trajectory]
z_trajectory = [list(z) for z in z_trajectory]

obstacles = [[5.0, 5.0, 5.0]]

overall_horizon_x = []
overall_horizon_y = []
overall_horizon_z = []

for x,y,z in zip(x_trajectory, y_trajectory, z_trajectory):
    overall_horizon_x.extend(x)
    overall_horizon_y.extend(y)
    overall_horizon_z.extend(z)

x_cmd, y_cmd, z_cmd = get_command_ref(df)

##### PLOTS #####
#plot the obstacles and the trajectory
fig,ax = plt.subplots()
ax.scatter(x_history, y_history, label='position')
ax.scatter(x_cmd, y_cmd, color='g', label='command')

for obstacle in obstacles:
    circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2]/2, color='r')
    ax.add_artist(circle)

ax.legend()
plt.show()

fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
for obstacle in obstacles:
    x = obstacle[0]
    y = obstacle[1]
    z = -10
    radius = obstacle[2]/2
    height = 10
    n_space = 10

    ax2.plot_surface(
        x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        z + height * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
        color='g',
        alpha=0.5
    )
ax2.plot(x_history, y_history, z_history, label='position', color='b')

#%%
### figure 3 ### 
fig3, ax3 = plt.subplots(subplot_kw=dict(projection="3d"))
ax3.set_xlim([min(x_cmd)-10, max(x_cmd)+10])
ax3.set_ylim([min(y_cmd)-10, max(y_cmd)+10])
ax3.set_zlim([min(z_cmd), max(z_cmd)])


for obstacle in obstacles:
    x = obstacle[0]
    y = obstacle[1]
    z = 20
    radius = obstacle[2]/2
    height = 10
    n_space = 10

    ax3.plot_surface(
        x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        z + height * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
        color='g',
        alpha=0.5
    )

actual_pos_array = np.array([x_history, y_history, z_history])
trajectory_data = np.array([overall_horizon_x, 
                            overall_horizon_y, 
                            overall_horizon_z])

position_data = [actual_pos_array, trajectory_data]
N = len(df['trajectory_x'][0])

labels = ['Actual Position', 'Horizon']

lines = [ax3.plot([], [], [])[0] for _ in range(len(position_data))] 
color_list = sns.color_palette("hls", len(lines))

for i, line in enumerate(lines):
    line._color = color_list[i]
    #line._color = color_map[uav_list[i]]
    line._linewidth = 5.0
    line.set_label(labels[i])
    
patches = lines
TIME_SPAN = 10

def init():
    lines = [ax3.plot(uav[0, 0:1], 
                      uav[1, 0:1], 
                      uav[2, 0:1])[0] for uav in position_data]
    return patches

def update_lines(num, dataLines, lines):
    count = 0 
    for i, (line, data) in enumerate(zip(lines, dataLines)):
        #NOTE: there is no .set_data() for 3 dim data...
        time_span = TIME_SPAN
        if num < time_span:
            interval = 0
        else:
            interval = num - time_span
            
        if i == 1:
            line.set_data(data[:2, N*num:N*num+N])
            line.set_3d_properties(data[2, N*num:N*num+N])
        else:
            line.set_data(data[:2, interval:num])
            line.set_3d_properties(data[2, interval:num])
            
        count +=1
    
    return patches

ax3.legend()
line_ani = FuncAnimation(fig3, update_lines, 
                fargs=(position_data, patches), init_func=init,
                interval=3, blit=True, 
                repeat=True, 
                frames=position_data[0].shape[1]+1)

# plt.show()