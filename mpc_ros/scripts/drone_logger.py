#!/usr/bin/env python3
import rclpy 
import numpy as np

from rclpy.node import Node

from mpc_ros import quaternion_tools, MPC, Config 
from mpc_ros.CasadiModels import SimpleQuadModel
from drone_ros.msg import Telem, CtlTraj

import pickle as pkl

import time 

class DroneLogger(Node):
    def __init__(self):
        super().__init__('drone_logger')
        
        self.frequency = 50
        self.trajectory_sub = self.create_subscription(CtlTraj, 'trajectory', 
                                self.trajectoryCallback, qos_profile=10 )

        self.telem_sub = self.create_subscription(Telem, 'telem',
                                self.telemetryCallback, qos_profile=10)
    
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.roll_history = []
        self.pitch_history = []
        self.yaw_history = []

        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_z = []
        self.trajectory_yaw = []
        self.trajectory_vx = []
        self.trajectory_vy = []
        self.trajectory_vz = []
        self.traj_yaw_rate = []
        self.idx = []
        self.flight_info = {}

    def telemetryCallback(self, msg: Telem):
        # print("x: ", msg.x)
        self.x_history.append(msg.x)
        self.y_history.append(msg.y)
        self.z_history.append(msg.z)
        self.roll_history.append(msg.roll)
        self.pitch_history.append(msg.pitch)
        self.yaw_history.append(msg.yaw)

    def trajectoryCallback(self, msg: CtlTraj):
        print("trajectory x: ", msg.x)
        self.trajectory_x.append(msg.x)
        self.trajectory_y.append(msg.y)
        self.trajectory_z.append(msg.z)
        self.trajectory_yaw.append(msg.yaw)
        self.trajectory_vx.append(msg.vx)
        self.trajectory_vy.append(msg.vy)
        self.trajectory_vz.append(msg.vz)
        self.traj_yaw_rate.append(msg.yaw_rate)
        self.idx.append(msg.idx)

    def saveData(self):
        self.flight_info['trajectory_x'] = self.trajectory_x
        self.flight_info['trajectory_y'] = self.trajectory_y
        self.flight_info['trajectory_z'] = self.trajectory_z
        self.flight_info['trajectory_yaw'] = self.trajectory_yaw
        self.flight_info['trajectory_vx'] = self.trajectory_vx
        self.flight_info['trajectory_vy'] = self.trajectory_vy
        self.flight_info['trajectory_vz'] = self.trajectory_vz
        self.flight_info['traj_yaw_rate'] = self.traj_yaw_rate
        self.flight_info['idx'] = self.idx

        self.flight_info['x_history'] = self.x_history
        self.flight_info['y_history'] = self.y_history
        self.flight_info['z_history'] = self.z_history
        self.flight_info['roll_history'] = self.roll_history
        self.flight_info['pitch_history'] = self.pitch_history
        self.flight_info['yaw_history'] = self.yaw_history

        with open('flight_info.pkl', 'wb') as f:
            pkl.dump(self.flight_info, f)
        
        print("Data saved")

def main(args=None):
    rclpy.init(args=args)
    drone_logger = DroneLogger()

    while rclpy.ok():
        
        try:
            rclpy.spin_once(drone_logger)
            
        #catch keyboard interrupt
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            drone_logger.saveData()
            
            rclpy.shutdown()
            break

if __name__ == '__main__':
    main()






