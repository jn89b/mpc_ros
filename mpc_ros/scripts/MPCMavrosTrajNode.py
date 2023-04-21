#!/usr/bin/env python3
import rclpy 
import numpy as np


from rclpy.node import Node

from mpc_ros import quaternion_tools, MPC, Config 
from mpc_ros.CasadiModels import SimpleQuadModel
from drone_ros.msg import Telem, CtlTraj
from mavros.base import SENSOR_QOS

import mavros
import pickle as pkl

import time 

class FlatQuadMPC(MPC.MPC):

    """this class is responsible for defining the MPC problem
    it inherits from the MPC class in mpc_ros package and overrides
    the addAdditionalConstraints method to add the constraints
    specific to the quadcopter
    """

    def __init__(self, mpc_params:dict, quad_constraint_params:dict):
        super().__init__(mpc_params)
        self.quad_constraint_params = quad_constraint_params

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
        #add additional constraints here
        self.lbx['U'][0,:] = self.quad_constraint_params['vx_min']
        self.ubx['U'][0,:] = self.quad_constraint_params['vx_max']

        self.lbx['U'][1,:] = self.quad_constraint_params['vy_min']
        self.ubx['U'][1,:] = self.quad_constraint_params['vy_max']

        self.lbx['U'][2,:] = self.quad_constraint_params['vz_min']
        self.ubx['U'][2,:] = self.quad_constraint_params['vz_max']

        self.lbx['U'][3,:] = self.quad_constraint_params['psi_dot_min']
        self.ubx['U'][3,:] = self.quad_constraint_params['psi_dot_max']

    def returnTrajDictionary(self, projected_controls:list,
        projected_states:list) -> dict:
        
        traj_dictionary = {}
        traj_dictionary['x'] = projected_states[0,:]
        traj_dictionary['y'] = projected_states[1,:]
        traj_dictionary['z'] = projected_states[2,:]
        traj_dictionary['yaw']= projected_states[3,:]

        traj_dictionary['vx'] = projected_controls[0,:]
        traj_dictionary['vy'] = projected_controls[1,:]
        traj_dictionary['vz'] = projected_controls[2,:]
        traj_dictionary['yaw_rate'] = projected_controls[3,:]

        return traj_dictionary


class MPCTrajPublisher(Node):
    """
    
    This node is responsible for publishing the trajectory from MPC
    to the controller node. It also subscribes to the state of the
    quadcopter and publishes the trajectory to the controller node.
    """

    def __init__(self):
        super().__init__('mpc_traj_node')

        self.get_logger().info("Starting MPC Trajectory Node")

        #turn this to a parameter later
        self.mpc_traj_freq = 50

        self.state_sub = self.create_subscription(Telem, 
                        'telem', self.stateCallback, 
                        self.mpc_traj_freq)

        self.state_sub = self.create_subscription(
            mavros.local_position.Odometry, 
            'mavros/local_position/odom', 
            self.stateCallback, 
            qos_profile=SENSOR_QOS)

        self.traj_pub = self.create_publisher(CtlTraj, 
                        'trajectory', 
                        self.mpc_traj_freq)

        self.state_info = [0, #x
                           0, #y
                           0, #z
                           0] #psi

        self.control_info = [0, #vx
                             0, #vy
                             0, #vz
                             0] #psi_dot

    def stateCallback(self, msg):
        #positions
        self.state_info[0] = msg.pose.pose.position.x
        self.state_info[1] = msg.pose.pose.position.y
        self.state_info[2] = msg.pose.pose.position.z

        #quaternion attitudes
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll,pitch,yaw = quaternion_tools.euler_from_quaternion(qx, 
                                                                qy, 
                                                                qz, 
                                                                qw)
        self.state_info[3] = (yaw+ (2*np.pi) ) % (2*np.pi);

        #velocities 
        self.control_info[0] = msg.twist.twist.linear.x
        self.control_info[1] = msg.twist.twist.linear.y
        self.control_info[2] = msg.twist.twist.linear.z

        #angular psi
        self.control_info[3] = msg.twist.twist.angular.z

    def computeError(self, current_state:list, desired_state:list) -> list:
        """computeError"""
        error = []
        for i in range(len(current_state)):
            error.append(desired_state[i] - current_state[i])
        return error

    def publishTrajectory(self, traj_dictionary:dict, 
                          state_idx:int, command_idx:int) -> None:
        """publishTrajectory"""
        traj_msg = CtlTraj()
        #x,y,z to one array
        traj_x = np.array(traj_dictionary['x'])
        traj_y = np.array(traj_dictionary['y'])
        traj_z = np.array(traj_dictionary['z'])
        traj_yaw = np.array(traj_dictionary['yaw'])

        traj_vx = np.array(traj_dictionary['vx'])
        traj_vy = np.array(traj_dictionary['vy'])
        traj_vz = np.array(traj_dictionary['vz'])
        traj_yaw_rate = np.array(traj_dictionary['yaw_rate'])
        

        #convert to list
        traj_x = traj_x[0].tolist()
        traj_y = traj_y[0].tolist()
        traj_z = traj_z[0].tolist()
        traj_yaw = traj_yaw[0].tolist()

        traj_vx = traj_vx[0].tolist()
        traj_vy = traj_vy[0].tolist()
        traj_vz = traj_vz[0].tolist()
        traj_yaw_rate = traj_yaw_rate[0].tolist()
        
        traj_msg = CtlTraj()
        #convert traj x to float
        traj_msg.x = [float(i) for i in traj_x]
        traj_msg.y = [float(i) for i in traj_y]
        traj_msg.z = [float(i) for i in traj_z]
        traj_msg.yaw = [float(i) for i in traj_yaw]

        #dz = traj_z - self.state_info[2]
        traj_msg.vx = [float(i) for i in traj_vx]
        traj_msg.vy = [float(i) for i in traj_vy]
        traj_msg.vz = [float(i) for i in traj_vz]
        traj_msg.yaw_rate = [float(i) for i in traj_yaw_rate]
        traj_msg.idx = state_idx

        self.traj_pub.publish(traj_msg)

def initQuadMPC():
    """this should be set as a parameter for the user"""
    #should allow user to map these parameters to a yaml file
    simple_quad_model = SimpleQuadModel.SimpleQuadModel()
    simple_quad_model.set_state_space()

    #set these as parameters 
    max_vel = 3.0;
    simple_quad_constraint_params = {
        'vx_max': max_vel, #m/s
        'vx_min': -max_vel, #m/s

        'vy_max': max_vel, #m/s
        'vy_min': -max_vel, #m/s

        'vz_max': 0.5, #m/s
        'vz_min': -0.5, #m/s

        'psi_dot_max': np.deg2rad(25.0),#rad/s
        'psi_dot_min': -np.deg2rad(25.0),#rad/s

        'z_min': 1, #m
        'z_max': 100, #m
    }

    simple_mpc_quad_params = {
        'model': simple_quad_model,
        'N': 25,
        'dt_val': 0.1,
        'Q': np.diag([1, 1, 1, 0.1]),
        'R': np.diag([1, 1, 1, 1.0])
    }

    quad_mpc = FlatQuadMPC(
        simple_mpc_quad_params, simple_quad_constraint_params)

    return quad_mpc


def get_state_control_ref(traj_dictionary:dict, 
    state_idx:int, ctrl_idx:int) -> tuple:
    """get_state_control_ref"""
    x_ref = traj_dictionary['x'][state_idx]
    y_ref = traj_dictionary['y'][state_idx]
    z_ref = traj_dictionary['z'][state_idx]
    psi_ref = traj_dictionary['yaw'][state_idx]

    vx_ref = traj_dictionary['vx'][ctrl_idx]
    vy_ref = traj_dictionary['vy'][ctrl_idx]
    vz_ref = traj_dictionary['vz'][ctrl_idx]
    psi_dot_ref = traj_dictionary['yaw_rate'][ctrl_idx]
    
    return [x_ref, y_ref, z_ref, psi_ref], \
        [vx_ref, vy_ref, vz_ref, psi_dot_ref]

def set_state_control_idx(mpc_params:dict, 
    solution_time:float, idx_buffer:int) -> int:
    """set index based on solution time"""
    time_rounded = round(solution_time, 1)
    
    if time_rounded <= 1:
        time_rounded = 1

    control_idx = mpc_params['dt_val']/time_rounded
    idx = int(round(control_idx)) + idx_buffer
    
    return idx

def isCollision(current_x:float, current_y:float) -> bool:
    """checkCollisionBubble"""
    #collision bubble radius
    r = Config.OBSTACLE_DIAMETER/2
    #collision bubble center
    x_c = Config.OBSTACLE_X
    y_c = Config.OBSTACLE_Y

    dist = np.sqrt((current_x - x_c)**2 + (current_y - y_c)**2)

    if dist <= r:
        print("Collision Detected", dist)
        return True
    else:
        return False

def main(args=None):
    rclpy.init(args=args)

    #turn this into a parameter later

    control_idx = 10
    state_idx = -2
    dist_error_tol = 5.0
    idx_buffer = 3

    quad_mpc = initQuadMPC()
    mpc_traj_node = MPCTrajPublisher()
    rclpy.spin_once(mpc_traj_node)

    goal_z = mpc_traj_node.state_info[2] #+ 10.0    

    desired_state = [Config.GOAL_X, 
                     Config.GOAL_Y, 
                     goal_z, 
                     mpc_traj_node.state_info[3]]

    #get the time to find solution 
    start_time = time.time()
    projected_controls, projected_states = quad_mpc.warmUpSolution(
        mpc_traj_node.state_info,
        desired_state, 
        mpc_traj_node.control_info)

    traj_dictionary = quad_mpc.returnTrajDictionary(
        projected_controls, projected_states)

    traj_state, traj_control = get_state_control_ref(
        traj_dictionary, state_idx, control_idx)

    end_time = time.time()

    control_idx = set_state_control_idx(quad_mpc.mpc_params, 
        end_time - start_time, idx_buffer=idx_buffer)

    state_idx = set_state_control_idx(quad_mpc.mpc_params,
        end_time - start_time, idx_buffer=idx_buffer)

    mpc_traj_node.publishTrajectory(traj_dictionary, state_idx, control_idx)

    while rclpy.ok():

        ref_state_error = mpc_traj_node.computeError(
            mpc_traj_node.state_info, traj_state)
          
        rclpy.spin_once(mpc_traj_node)
        offset_state = [mpc_traj_node.state_info[0] + ref_state_error[0]/1.5,
                        mpc_traj_node.state_info[1] + ref_state_error[1]/1.5,
                        mpc_traj_node.state_info[2] + ref_state_error[2]/1.5,
                        mpc_traj_node.state_info[3] + ref_state_error[3]/1.5]

        quad_mpc.reinitStartGoal(offset_state, desired_state)
        start_time = time.time()

        projected_controls, projected_states = quad_mpc.solveMPCRealTimeStatic(
            offset_state,
            desired_state,
            mpc_traj_node.control_info)
        end_time = time.time()

        control_idx = set_state_control_idx(quad_mpc.mpc_params, 
            end_time - start_time, idx_buffer=idx_buffer)

        state_idx = set_state_control_idx(quad_mpc.mpc_params,
            end_time - start_time, idx_buffer=idx_buffer)

        traj_dictionary = quad_mpc.returnTrajDictionary(
            projected_controls, projected_states)

        traj_state, traj_control = get_state_control_ref(
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
            rclpy.spin_once(mpc_traj_node)

            current_x = mpc_traj_node.state_info[0]
            current_y = mpc_traj_node.state_info[1]

            if isCollision(current_x, current_y) == True:
                #send 0 velocity command
                ref_state_error = [0.0, 0.0, 0.0, 0.0]
                ref_control_error = [0.0, 0.0, 0.0, 0.0]

                mpc_traj_node.publishTrajectory(traj_dictionary, 
                                                state_idx, 
                                                control_idx)


                # mpc_traj_node.destroy_node()
                # rclpy.shutdown()
                # return

        if distance_error < dist_error_tol:
            """
            refactor add the length of N of commands to 
            stay where you are at 
            """
            #send 0 velocity command
            # ref_state_error = [0.0, 0.0, 0.0, 0.0]
            # ref_control_error = [0.0, 0.0, 0.0, 0.0]

            #set trajectory to stay at current position

            end_number = 100
            state_idx = end_number
            control_idx = end_number

            mpc_traj_node.publishTrajectory(traj_dictionary, 
                                            state_idx, 
                                            control_idx)
            

            # mpc_traj_node.destroy_node()
            # rclpy.shutdown()
            # return 

        desired_state = [Config.GOAL_X, 
                         Config.GOAL_Y, 
                         goal_z, 
                         offset_state[3]]

        rclpy.spin_once(mpc_traj_node)

if __name__ == '__main__':
    main()



        

