#!/usr/bin/env python3
import casadi as ca
import rclpy 
import numpy as np

from drone_interfaces.msg import Telem, CtlTraj
from mpc_ros.CasadiModels.AirplaneModel import AirplaneSimpleModel
from mpc_ros.MPC import MPC
from mpc_ros import RadarNetwork
from rclpy.node import Node
from mpc_ros import quaternion_tools, Config 

import pickle as pkl

import time

import mavros
from mavros.base import SENSOR_QOS

class Effector():
    def __init__(self, effector_range:float):
        self.effector_range = effector_range
        self.effector_power = 10

    def computePowerDensity(self,target_distance,
                            effector_factor):
        return self.effector_power * effector_factor/ (target_distance**2 * 4*ca.pi)

class AirplaneSimpleModelMPC(MPC):
    def __init__(self, mpc_params:dict, 
                 airplane_constraint_params:dict):
        super().__init__(mpc_params)
        self.airplane_params = airplane_constraint_params
        self.S = 0.5
        
        if 'effector' in self.airplane_params.keys():
            self.effector = self.airplane_params['effector']
            self.effector_range = self.effector.effector_range
            self.T = 0.1
        else:
            self.effector = None

        if Config.RADAR_AVOID == True:
            self.radar = RadarNetwork.RadarNetwork()
            
            self.radar_weight = 10
            self.radar_history = []
            
    def computeCost(self):
        #tired of writing self
        #dynamic constraints 

        P = self.P
        Q = self.Q
        R = self.R
        n_states = self.n_states
        
        for k in range(self.N):
            states = self.X[:, k]
            controls = self.U[:, k]
            state_next = self.X[:, k+1]
            
            #penalize states and controls for now, can add other stuff too
            self.cost_fn = self.cost_fn \
                + (states - P[n_states:]).T @ Q @ (states - P[n_states:]) \
                + controls.T @ R @ controls                 

            # self.cost_fn =             
            ##Runge Kutta
            k1 = self.f(states, controls)
            k2 = self.f(states + self.dt_val/2*k1, controls)
            k3 = self.f(states + self.dt_val/2*k2, controls)
            k4 = self.f(states + self.dt_val * k3, controls)
            state_next_RK4 = states + (self.dt_val / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            self.g = ca.vertcat(self.g, state_next - state_next_RK4) #dynamic constraints
      
            if self.effector != None:
                x_pos = self.X[0,k]
                y_pos = self.X[1,k]
                z_pos = self.X[2,k]
                phi = self.X[3,k]
                theta = self.X[4,k]
                psi = self.X[5,k]

                dx = Config.GOAL_X - x_pos
                dy = Config.GOAL_Y - y_pos
                dz = Config.GOAL_Z - z_pos

                dtarget = ca.sqrt(dx**2 + dy**2 + dz**2)
                
                error_dist_factor = 1 - (dtarget / self.effector_range)
                
                effector_dmg = self.effector.computePowerDensity(
                    dtarget, error_dist_factor)
                
                #minus because we want to maximize damage
                self.cost_fn = self.cost_fn + (self.T* effector_dmg)

        if Config.RADAR_AVOID == True:
            for k in range(self.N):
                current_position = self.X[:3,k]
                current_heading = self.X[5,k]
                current_roll = self.X[4,k]

                detection_vals, prob_detection = self.radar.get_detection_probability(
                    current_position, current_heading, current_roll)

                self.cost_fn = self.cost_fn + (self.radar_weight * prob_detection)
                self.radar_history.append(detection_vals)

                #threshold
                detection_threshold = 0.5
                radar_constraint = prob_detection + detection_threshold

                self.g = ca.vertcat(self.g, radar_constraint)
                
        if Config.OBSTACLE_AVOID:
            for k in range(self.N):
                #penalize obtacle distance
                x_pos = self.X[0,k]
                y_pos = self.X[1,k]                
                obs_distance = ca.sqrt((x_pos - Config.OBSTACLE_X)**2 + \
                                        (y_pos - Config.OBSTACLE_Y)**2) 

    
                obs_constraint = -obs_distance + ((Config.ROBOT_DIAMETER/2) + \
                    (Config.OBSTACLE_DIAMETER/2))

                self.cost_fn = self.cost_fn - (self.S* obs_distance)

                self.g = ca.vertcat(self.g, obs_constraint) 

        if Config.MULTIPLE_OBSTACLE_AVOID:
            for obstacle in Config.OBSTACLES:
                obs_x = obstacle[0]
                obs_y = obstacle[1]
                obs_diameter = obstacle[2]

                for k in range(self.N):
                    #penalize obtacle distance
                    x_pos = self.X[0,k]
                    y_pos = self.X[1,k]                
                    obs_distance = ca.sqrt((x_pos - obs_x)**2 + \
                                            (y_pos - obs_y)**2)
                    

                    obs_constraint = -obs_distance + ((Config.ROBOT_DIAMETER) + \
                        (obs_diameter/2)) 

                    self.g = ca.vertcat(self.g, obs_constraint)
                
                    if obstacle == [Config.GOAL_X, Config.GOAL_Y]:
                        continue
                    
                    self.cost_fn = self.cost_fn + (self.S* obs_distance)


    def warmUpSolution(self, start:list, goal:list, controls:list) -> tuple:
        self.initDecisionVariables()
        self.defineBoundaryConstraints()
        self.addAdditionalConstraints()
        self.reinitStartGoal(start, goal)
        self.computeCost()
        self.initSolver()

        projected_controls,projected_states = self.solveMPCRealTimeStatic(
            start,goal, controls)
        
        return projected_controls,projected_states

    def addAdditionalConstraints(self) -> None:
        """add additional constraints to the MPC problem"""
        # add control constraints
        self.lbx['U'][0, :] = self.airplane_params['u_phi_min']
        self.ubx['U'][0, :] = self.airplane_params['u_phi_max']

        self.lbx['U'][1, :] = self.airplane_params['u_theta_min']
        self.ubx['U'][1, :] = self.airplane_params['u_theta_max']

        self.lbx['U'][2, :] = self.airplane_params['u_psi_min']
        self.ubx['U'][2, :] = self.airplane_params['u_psi_max']

        self.lbx['U'][3, :] = self.airplane_params['v_cmd_min']
        self.ubx['U'][3, :] = self.airplane_params['v_cmd_max']

        self.lbx['X'][2, :] = self.airplane_params['z_min']
        # self.ubx['X'][2,:] = self.airplane_params['z_max']

        self.lbx['X'][3, :] = self.airplane_params['phi_min']
        self.ubx['X'][3, :] = self.airplane_params['phi_max']

        self.lbx['X'][4, :] = self.airplane_params['theta_min']
        self.ubx['X'][4, :] = self.airplane_params['theta_max']

    def returnTrajDictionary(self, 
        projected_controls:list,
        projected_states:list) -> dict:
        traj_dictionary = {}
        traj_dictionary['x'] = projected_states[0,:]
        traj_dictionary['y'] = projected_states[1,:]
        traj_dictionary['z'] = projected_states[2,:]
        traj_dictionary['phi']= projected_states[3,:]
        traj_dictionary['theta'] = projected_states[4,:]
        traj_dictionary['psi'] = projected_states[5,:]
        traj_dictionary['v'] = projected_states[6,:]

        traj_dictionary['u_phi'] = projected_controls[0,:]
        traj_dictionary['u_theta'] = projected_controls[1,:]
        traj_dictionary['u_psi'] = projected_controls[2,:]
        traj_dictionary['v_cmd'] = projected_controls[3,:]

        return traj_dictionary
    
    def get_state_control_ref(self, 
        traj_dictionary:dict, 
        state_idx:int, 
        ctrl_idx:int) -> tuple:
        """get_state_control_ref"""
        x_ref = traj_dictionary['x'][state_idx]
        y_ref = traj_dictionary['y'][state_idx]
        z_ref = traj_dictionary['z'][state_idx]
        phi_ref = traj_dictionary['phi'][state_idx]
        theta_ref = traj_dictionary['theta'][state_idx]
        psi_ref = traj_dictionary['psi'][state_idx]
        vel_ref = traj_dictionary['v'][state_idx]

        u_phi_ref = traj_dictionary['u_phi'][ctrl_idx]
        u_theta_ref = traj_dictionary['u_theta'][ctrl_idx]
        u_psi_ref = traj_dictionary['u_psi'][ctrl_idx]
        v_cmd_ref = traj_dictionary['v_cmd'][ctrl_idx]

        return [x_ref, y_ref, z_ref, phi_ref, theta_ref, psi_ref, vel_ref], \
            [u_phi_ref, u_theta_ref, u_psi_ref, v_cmd_ref]

    def set_state_control_idx(self, mpc_params:dict, 
        solution_time:float, idx_buffer:int) -> int:
        """set index based on solution time"""
        time_rounded = round(solution_time, 1)
        
        if time_rounded <= 1:
            time_rounded = 1

        control_idx = mpc_params['dt_val']/time_rounded
        idx = int(round(control_idx)) + idx_buffer
        
        return idx


class MPCTrajFWPublisher(Node):
    def __init__(self):
        super().__init__('mpc_traj_fw_publisher')

        self.get_logger().info('Starting MPC Traj FW Publisher')


        #turn this to a parameter later
        self.mpc_traj_freq = 100
        
        # self.state_info = [0,0,0,0,0,0,0,0] #x, y, z, psi, vx, vy, vz, psi_dot
        self.state_info = [0, # x
                           0, # y
                           0, # z
                           0, # phi
                           0, # theta
                           0, # psi
                           0  # airspeed
                           ]
        
        self.control_info = [0, # u_phi
                             0, # u_theta
                             0, # u_psi
                             0  # v_cmd
                            ]
        
        self.traj_pub = self.create_publisher(CtlTraj, 
                        'trajectory', 
                        self.mpc_traj_freq)

        self.state_sub = self.create_subscription(mavros.local_position.Odometry,
                                                  'mavros/local_position/odom', 
                                                  self.position_callback, 
                                                  qos_profile=SENSOR_QOS)

        self.initHistory()

    def initHistory(self) -> None:
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.phi_history = []
        self.theta_history = []
        self.psi_history = []
        self.v_history = []

        self.x_trajectory = []
        self.y_trajectory = []
        self.z_trajectory = []
        self.phi_trajectory = []
        self.theta_trajectory = []
        self.psi_trajectory = []
        self.idx_history = []
        self.v_trajectory = []
        self.obstacles = []
        self.radar_probability = []


        self.pickle_history = {
            'x_history': None,
            'y_history': None,
            'z_history': None,

            'phi_history': None,
            'theta_history': None,
            'psi_history': None,
            'v_history': None,

            'x_trajectory': None,
            'y_trajectory': None,
            'z_trajectory': None,
            'phi_trajectory': None,
            'theta_trajectory': None,
            'psi_trajectory': None,
            'v_trajectory': None,
            'idx_history': None,
            'obstacles': None
        }

    def savePickle(self, pickle_file:str) -> None:
        """save pickle file"""
        self.pickle_history['x_history'] = self.x_history
        self.pickle_history['y_history'] = self.y_history
        self.pickle_history['z_history'] = self.z_history

        self.pickle_history['phi_history'] = self.phi_history
        self.pickle_history['theta_history'] = self.theta_history
        self.pickle_history['psi_history'] = self.psi_history
        self.pickle_history['v_history'] = self.v_history

        self.pickle_history['x_trajectory'] = self.x_trajectory
        self.pickle_history['y_trajectory'] = self.y_trajectory
        self.pickle_history['z_trajectory'] = self.z_trajectory
        self.pickle_history['phi_trajectory'] = self.phi_trajectory
        self.pickle_history['theta_trajectory'] = self.theta_trajectory
        self.pickle_history['psi_trajectory'] = self.psi_trajectory
        self.pickle_history['v_trajectory'] = self.v_trajectory
        self.pickle_history['idx_history'] = self.idx_history

        #check if config Obstacles is None
        #self.pickle_history['obstacles'] = Config.OBSTACLES

        with open(pickle_file+'.pkl', 'wb') as handle:
            pkl.dump(self.pickle_history, handle)

        print('Saved pickle file: ', pickle_file)

    # def stateCallback(self, msg):
    #     enu_coords = quaternion_tools.convertNEDToENU(
    #         msg.x, msg.y, msg.z)
    #     # positions
    #     self.state_info[0] = enu_coords[0]
    #     self.state_info[1] = enu_coords[1]
    #     self.state_info[2] = enu_coords[2]

    #     #wrap yaw to 0-360
    #     self.state_info[3] = msg.roll
    #     self.state_info[4] = msg.pitch
    #     self.state_info[5] = msg.yaw #-(msg.yaw+ (2*np.pi) ) % (2*np.pi);
    #     self.state_info[6] = np.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)

    #     #rotate roll and pitch rates to ENU frame   
    #     self.control_info[0] = msg.roll_rate
    #     self.control_info[1] = msg.pitch_rate
    #     self.control_info[2] = msg.yaw_rate
    #     self.control_info[3] = np.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)

    #     print("yaw", np.rad2deg(msg.yaw)) 
    #     #print("atan2", np.rad2deg(np.arctan2(enu_coords[1], enu_coords[0])))

    def position_callback(self, msg):
        # positions
        self.state_info[0] = msg.pose.pose.position.x
        self.state_info[1] = msg.pose.pose.position.y
        self.state_info[2] = msg.pose.pose.position.z

        # quaternion attitudes
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = quaternion_tools.euler_from_quaternion(
            qx, qy, qz, qw)

        self.state_info[3] = roll
        self.state_info[4] = pitch
        self.state_info[5] = yaw  # (yaw+ (2*np.pi) ) % (2*np.pi);
        # wr

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        #get magnitude of velocity
        self.state_info[6] = np.sqrt(vx**2 + vy**2 + vz**2)
        #self.state_info[6] = #msg.twist.twist.linear.x
        self.control_info[0] = msg.twist.twist.angular.x
        self.control_info[1] = msg.twist.twist.angular.y
        self.control_info[2] = msg.twist.twist.angular.z
        self.control_info[3] = msg.twist.twist.linear.x
    
        self.x_history.append(self.state_info[0])
        self.y_history.append(self.state_info[1])
        self.z_history.append(self.state_info[2])
        self.phi_history.append(self.state_info[3])
        self.theta_history.append(self.state_info[4])
        self.psi_history.append(self.state_info[5])
        self.v_history.append(self.state_info[6])

    def computeError(self, current_state:list, desired_state:list) -> list:
        """computeError"""
        error = []
        #catch list index out of range
        for i in range(len(current_state)):
            error.append(desired_state[i] - current_state[i])
        return error
    
    def publishTrajectory(self, traj_dictionary:dict, 
                          state_idx:int, command_idx:int) -> None:
        """
        publishTrajectory
        
        Watch out for the sign convention changes between ENU and NED
        """

        traj_x = np.array(traj_dictionary['x'])
        traj_y = np.array(traj_dictionary['y'])
        traj_z = np.array(traj_dictionary['z'])
        
        traj_phi = np.array(traj_dictionary['phi'])
        traj_theta = -np.array(traj_dictionary['theta'])
        traj_psi = np.array(traj_dictionary['psi'])

        traj_u_phi = np.array(traj_dictionary['u_phi'])
        traj_u_theta = -np.array(traj_dictionary['u_theta'])
        traj_u_psi = np.array(traj_dictionary['u_psi'])
        traj_v_cmd = np.array(traj_dictionary['v_cmd'])
        
        ned_position = quaternion_tools.convertENUToNEDVector(
            traj_x, traj_y, traj_z)
        
        traj_msg = CtlTraj()
        traj_msg.x = list(ned_position[0][0])
        traj_msg.y = list(ned_position[1][0])
        traj_msg.z = list(ned_position[2][0])
        
        traj_msg.roll = list(traj_phi[0])
        traj_msg.pitch = list(traj_theta[0])
        traj_msg.yaw = list(traj_psi[0])
        
        traj_msg.roll_rate = list(traj_u_phi[0])
        traj_msg.pitch_rate = list(traj_u_theta[0])
        traj_msg.yaw_rate = list(traj_u_psi[0])
        traj_msg.vx = list(traj_v_cmd[0])
        
        traj_msg.idx = state_idx

        self.x_trajectory.append(traj_x)
        self.y_trajectory.append(traj_y)
        self.z_trajectory.append(traj_z)
        self.v_trajectory.append(traj_v_cmd)
        self.phi_trajectory.append(traj_phi)
        self.theta_trajectory.append(traj_theta)
        self.psi_trajectory.append(traj_psi)
        self.idx_history.append(state_idx)
        self.traj_pub.publish(traj_msg)


def initFWMPC() -> AirplaneSimpleModelMPC:

    simple_airplane_model = AirplaneSimpleModel()
    simple_airplane_model.set_state_space()
    
    airplane_params = {
        'u_psi_min': np.deg2rad(-45), #rates
        'u_psi_max': np.deg2rad(45), #
        'u_phi_min': np.deg2rad(-45),
        'u_phi_max': np.deg2rad(45),
        'u_theta_min': np.deg2rad(-10),
        'u_theta_max': np.deg2rad(10),
        'z_min': 5.0,
        'z_max': 100.0,
        'v_cmd_min': 18,
        'v_cmd_max': 22,
        'theta_min': np.deg2rad(-10),
        'theta_max': np.deg2rad(10),
        'phi_min': np.deg2rad(-45),
        'phi_max': np.deg2rad(45),
        'effector': Effector(effector_range=10)
    }
    
    Q = ca.diag([1.0, 1.0, 0.75, 1.0, 1.0, 1.0, 1.0])
    R = ca.diag([0.5, 1.0, 1.0, 1.0])

    simple_mpc_fw_params = {
        'model': simple_airplane_model,
        'dt_val': 0.1,
        'N': 25,
        'Q': Q,
        'R': R
    }

    fw_mpc = AirplaneSimpleModelMPC(simple_mpc_fw_params, 
                                    airplane_params)

    return fw_mpc

def main(args=None):
    rclpy.init(args=args)

    control_idx = 10
    state_idx = 5
    dist_error_tol = 5.0
    idx_buffer = 2

    fw_mpc = initFWMPC()
    mpc_traj_node = MPCTrajFWPublisher()
    rclpy.spin_once(mpc_traj_node)

    goal_z = 45
    dist_error_tol = 20

    desired_state = [
            Config.GOAL_X, 
            Config.GOAL_Y,
            goal_z, 
            0, 
            0, 
            0, 
            15]
    print("len of desired state: ", len(desired_state))

    #get the time to find solution 
    start_time = time.time()
    projected_controls, projected_states = fw_mpc.warmUpSolution(
        mpc_traj_node.state_info,
        desired_state, 
        mpc_traj_node.control_info)

    traj_dictionary = fw_mpc.returnTrajDictionary(
        projected_controls, projected_states)

    traj_state, traj_control = fw_mpc.get_state_control_ref(
        traj_dictionary, state_idx, control_idx)

    print("traj state: ", traj_state)
    end_time = time.time()

    control_idx = fw_mpc.set_state_control_idx(fw_mpc.mpc_params, 
        end_time - start_time, idx_buffer)

    state_idx = fw_mpc.set_state_control_idx(fw_mpc.mpc_params,
        end_time - start_time, idx_buffer)

    mpc_traj_node.publishTrajectory(traj_dictionary, state_idx, control_idx)

    while rclpy.ok():
        print("state info: ", mpc_traj_node.state_info)
        ref_state_error = mpc_traj_node.computeError(
            mpc_traj_node.state_info, traj_state)
          
        rclpy.spin_once(mpc_traj_node)
        
        #predict the next state
        offset_state = [mpc_traj_node.state_info[0], #+ ref_state_error[0]/1.5,
                        mpc_traj_node.state_info[1], #+ ref_state_error[1]/1.5,
                        mpc_traj_node.state_info[2], #+ ref_state_error[2]/1.5,
                        mpc_traj_node.state_info[3], #+ ref_state_error[3]/1.5, 
                        mpc_traj_node.state_info[4], #+ ref_state_error[4]/1.5,
                        mpc_traj_node.state_info[5], #+ ref_state_error[5]/1.5, 
                        mpc_traj_node.state_info[6]] #+ ref_state_error[6]/1.5]
        
        fw_mpc.reinitStartGoal(offset_state, desired_state)
        start_time = time.time()

        projected_controls, projected_states = fw_mpc.solveMPCRealTimeStatic(
            offset_state,
            desired_state,
            mpc_traj_node.control_info)
        end_time = time.time()

        control_idx = fw_mpc.set_state_control_idx(fw_mpc.mpc_params, 
            end_time - start_time, idx_buffer=idx_buffer)

        state_idx = fw_mpc.set_state_control_idx(fw_mpc.mpc_params,
            end_time - start_time, idx_buffer=idx_buffer)

        traj_dictionary = fw_mpc.returnTrajDictionary(
            projected_controls, projected_states)

        traj_state, traj_control = fw_mpc.get_state_control_ref(
            traj_dictionary, state_idx, control_idx)

        mpc_traj_node.publishTrajectory(traj_dictionary, 
                                        state_idx, 
                                        control_idx)

        goal_state_error = mpc_traj_node.computeError(
            mpc_traj_node.state_info, desired_state)

        distance_error = np.linalg.norm(goal_state_error[0:2])
        
        print("distance error: ", distance_error)
        print("\n")

        if Config.OBSTACLE_AVOID:
            #check if within obstacle SD
            # rclpy.spin_once(mpc_traj_node)

            current_x = mpc_traj_node.state_info[0]
            current_y = mpc_traj_node.state_info[1]

            # if isCollision(current_x, current_y) == True:

            #     print("trajectory x", traj_dictionary['x'])
            #     print("trajectory y", traj_dictionary['y'])
            #     #send 0 velocity command
            #     ref_state_error = [0.0, 0.0, 0.0, 0.0]
            #     ref_control_error = [0.0, 0.0, 0.0, 0.0]

            #     mpc_traj_node.publishTrajectory(traj_dictionary, 
            #                                     state_idx, 
            #                                     control_idx)


            #     # mpc_traj_node.destroy_node()
            #     # rclpy.shutdown()
            #     # return

        if distance_error < dist_error_tol:
            """
            refactor add the length of N of commands to 
            stay where you are at 
            """
            #send 0 velocity command
            # ref_state_error = [0.0, 0.0, 0.0, 0.0]
            # ref_control_error = [0.0, 0.0, 0.0, 0.0]
            mpc_traj_node.savePickle('test')
            mpc_traj_node.publishTrajectory(traj_dictionary, 
                                            state_idx, 
                                            control_idx)


            mpc_traj_node.destroy_node()
            rclpy.shutdown()
            return 

        desired_state = [
                Config.GOAL_X, 
                Config.GOAL_Y,
                goal_z, 
                mpc_traj_node.state_info[3], 
                mpc_traj_node.state_info[4], 
                mpc_traj_node.state_info[5], 
                mpc_traj_node.state_info[6]]

        # rclpy.spin_once(mpc_traj_node)

if __name__ == '__main__':
    main()
