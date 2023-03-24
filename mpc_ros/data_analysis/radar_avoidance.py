import sys
sys.path.append("..") # Adds higher directory to python modules path.

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os 
import ast
import numpy as np
from matplotlib.animation import FuncAnimation
import pickle as pkl

from mpc_ros import RadarNetwork


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
single_radar = 'test'
single_big = 'single_big_radar'
loud_front = 'loud_front_mpc'
with open(single_big+'.pkl', 'rb') as f:
    df = pkl.load(f)

radar = RadarNetwork.RadarNetwork()
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
# ax.scatter(x_cmd, y_cmd, color='g', label='command')

for individ_radar in radar.radars:
    x_pos = individ_radar.position[0]
    y_pos = individ_radar.position[1]
    radius = individ_radar.range
    circle = plt.Circle((x_pos, y_pos), radius, color='r', alpha=0.5)
    ax.add_artist(circle)

ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title('Radar Avoidance')
ax.legend()

# for obstacle in obstacles:
#     circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2]/2, color='r')
#     ax.add_artist(circle)

ax.legend()
plt.show()

fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
# for obstacle in obstacles:
#     x = obstacle[0]
#     y = obstacle[1]
#     z = 0
#     radius = obstacle[2]/2
#     height = 50
#     n_space = 10
for individ_radar in radar.radars:
    x_pos = individ_radar.position[0]
    y_pos = individ_radar.position[1]
    z_pos = individ_radar.position[2]
    radius = individ_radar.range

   #plot a 3D circle
    n_space = 30

    # Create a grid of points in the x, y, and z directions
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = x_pos+radius*np.cos(u)*np.sin(v)
    y = y_pos+radius*np.sin(u)*np.sin(v)
    z = z_pos+ 100*np.cos(v)

    ax2.plot_surface(x, y, z, 
                     color="r",
                     alpha=0.5)
    
    ax2.set_zlim3d(0, 100)
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.set_zlabel('z (m)')


    # ax2.plot_surface(
    #     x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
    #     y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
    #     z + radius * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
    #     color='r',
    #     alpha=0.5
    # )

#     ax2.plot_surface(
#         x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
#         y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
#         z + height * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
#         color='g',
#         alpha=0.5
#     )
ax2.plot(x_history, y_history, z_history, label='position', color='b')



#%%
### figure 3 ### 
fig3, ax3 = plt.subplots(subplot_kw=dict(projection="3d"))
ax3.set_xlim([min(x_cmd), max(x_cmd)])
ax3.set_ylim([min(y_cmd), max(y_cmd)])
ax3.set_zlim([min(z_cmd), max(z_cmd)])


# for obstacle in obstacles:
#     x = obstacle[0]
#     y = obstacle[1]
#     z = 20
#     radius = obstacle[2]/2
#     height = 30
#     n_space = 10

#     ax3.plot_surface(
#         x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
#         y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
#         z + height * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
#         color='g',
#         alpha=0.5
#     )


#%% Plot Radar Avoidance
radar_history = []
distances_from_radar = []
relative_headings = []
detection_value_history = []
prob_detections = []

for i in range(len(x_trajectory)-1):
    
    target_position = [x_history[i], y_history[i], z_history[i]]
    psi = psi_history[i]
    phi = phi_history[i]

    distance_from_radar,relative_headings, relative_elevations  = radar.get_distance_headings(
        target_position, psi,phi,
        use_casadi=False)
    
    detection_values = radar.get_detection_values(distance_from_radar, relative_headings, 
                                                    relative_elevations, 
                                                    use_casadi=False)
    
    prob_detection = radar.compute_radar_probability(detection_values, use_casadi=False)
    print(prob_detection)

    distances_from_radar.append(distance_from_radar)
    relative_headings.append(relative_headings)
    detection_value_history.append(detection_values)
    prob_detections.append(prob_detection)

#print("prob_detections: ", prob_detections)



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

#plot attitudes
t_history = np.arange(0, len(phi_history))
fig4, ax4 = plt.subplots(3,1)
ax4[0].plot(t_history, np.rad2deg(phi_history), label='phi')
ax4[1].plot(t_history, np.rad2deg(theta_history), label='psi')
ax4[2].plot(t_history, np.rad2deg(psi_history), label='theta')

ax4[0].legend()
ax4[1].legend()
ax4[2].legend()

#%% Plot Radar Avoidance
fig5, ax5 = plt.subplots(1)
ax5.plot(t_history[:-1], prob_detections, label='Radar Detection')
#plot horizontal line at 0.5
# ax5.axhline(y=0.5, color='r', linestyle='--', label='detection threshold')
ax5.legend()
ax5.set_xlabel('time (s)')
ax5.set_ylabel('Probability of Detection')
