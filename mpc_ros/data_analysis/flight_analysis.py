import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os 
import ast
import numpy as np
from matplotlib.animation import FuncAnimation
import pickle as pkl

plt.close('all')

def get_command_ref(df:dict):
    x_traj = df['x_trajectory']
    y_traj = df['y_trajectory']
    z_traj = df['z_trajectory']
    phi_traj = df['phi_trajectory']
    theta_traj = df['theta_trajectory']
    psi_traj = df['psi_trajectory']
    idx_history = df['idx_history']

    x_cmd = []
    y_cmd = []
    z_cmd = []
    phi_cmd = []
    theta_cmd = []
    psi_cmd = []

    for x,y,z,phi,theta,psi,idx in zip(x_traj, 
                                       y_traj, 
                                       z_traj, 
                                       phi_traj, 
                                       theta_traj, 
                                       psi_traj, 
                                       idx_history):
        
        x_cmd.append(x[0][idx])
        y_cmd.append(y[0][idx])
        z_cmd.append(z[0][idx])
        phi_cmd.append(phi[0][idx])
        theta_cmd.append(theta[0][idx])
        psi_cmd.append(psi[0][idx])
    
    return x_cmd, y_cmd, z_cmd, phi_cmd, theta_cmd, psi_cmd


#load pickle file
with open('test.pkl', 'rb') as f:
    df = pkl.load(f)


x_history = df['x_history']
y_history = df['y_history']
z_history = df['z_history']
phi_history = df['phi_history']
theta_history = df['theta_history']
psi_history = df['psi_history']
obstacles = df['obstacles']


x_trajectory = df['x_trajectory']
y_trajectory = df['y_trajectory']
z_trajectory = df['z_trajectory']
x_trajectory = [x[0] for x in x_trajectory]
y_trajectory = [y[0] for y in y_trajectory]
z_trajectory = [z[0] for z in z_trajectory]

overall_horizon_x = []
overall_horizon_y = []
overall_horizon_z = []

for x,y,z in zip(x_trajectory, y_trajectory, z_trajectory):
    overall_horizon_x.extend(x)
    overall_horizon_y.extend(y)
    overall_horizon_z.extend(z)

x_cmd, y_cmd, z_cmd, phi_cmd, theta_cmd, psi_cmd = get_command_ref(df)

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
    z = 0
    radius = obstacle[2]/2
    height = 50
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
ax3.set_xlim([min(x_cmd), max(x_cmd)])
ax3.set_ylim([min(y_cmd), max(y_cmd)])
ax3.set_zlim([min(z_cmd), max(z_cmd)])

for obstacle in obstacles:
    x = obstacle[0]
    y = obstacle[1]
    z = 20
    radius = obstacle[2]/2
    height = 30
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
N = len(df['x_trajectory'][0][0])

labels = ['Actual Position', 'Horizon']

lines = [ax3.plot([], [], [])[0] for _ in range(len(position_data))] 
color_list = sns.color_palette("hls", len(lines))

for i, line in enumerate(lines):
    line._color = color_list[i]
    #line._color = color_map[uav_list[i]]
    line._linewidth = 5.0
    line.set_label(labels[i])
    
patches = lines
TIME_SPAN = 20

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
                interval=30, blit=True, 
                repeat=True, 
                frames=position_data[0].shape[1]+1)

# plt.show()