#!/usr/bin/env python3

import rclpy
import math as m
from drone_interfaces.msg import Telem,CtlTraj

from pymavlink import mavutil
from rclpy.node import Node
import time
import numpy as np

class MAVListener :
    def __init__(self, master, frequency=5):
        self.master = master
        self.frequency = frequency

    def startListening(self):
        self.requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 
                                    self.frequency)
        self.requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                                    self.frequency)
        self.requestMessageInterval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                    self.frequency)
    
    def getData(self):
        
        output = Telem()
        msg = self.master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE', 'GLOBAL_POSITION_INT'], blocking=True)

        try:
            output.lat = self.master.messages['GLOBAL_POSITION_INT'].lat
            output.lon = self.master.messages['GLOBAL_POSITION_INT'].lon
            output.alt = self.master.messages['GLOBAL_POSITION_INT'].alt
            output.heading = self.master.messages['GLOBAL_POSITION_INT'].hdg

            output.roll = self.master.messages['ATTITUDE'].roll
            output.pitch = self.master.messages['ATTITUDE'].pitch
            output.yaw = self.master.messages['ATTITUDE'].yaw

            output.x = self.master.messages['LOCAL_POSITION_NED'].x
            output.y = self.master.messages['LOCAL_POSITION_NED'].y
            output.z = self.master.messages['LOCAL_POSITION_NED'].z
            output.vx = self.master.messages['LOCAL_POSITION_NED'].vx
            output.vy = self.master.messages['LOCAL_POSITION_NED'].vy
            output.vz = self.master.messages['LOCAL_POSITION_NED'].vz
        
        #catch key error 
        except KeyError:
            return output

        return output 

    def requestMessageInterval(self, message_id:int, frequency_hz:int):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

class DroneNode(Node):
    def __init__(self, master):
        super().__init__('drone_node')
        
        self.get_logger().info("Drone node started")

        self.frequency = 100
        self.master = master
        self.mav_listener = MAVListener(self.master, self.frequency)
        self.mav_listener.startListening()
        self.telem_pub = self.create_publisher(Telem, 'telem', self.frequency)

        
        self.trajectory_sub = self.create_subscription(CtlTraj, 'trajectory', 
                                self.trajectoryCallback, self.frequency)

    def publishData(self):
        self.mav_listener.startListening()
        output = self.mav_listener.getData()
        self.telem_pub.publish(output)

    def trajectoryCallback(self,msg:CtlTraj):
        cmd_x = msg.x
        cmd_y = msg.y
        cmd_z = msg.z
        cmd_yaw = msg.yaw

        cmd_vx = msg.vx
        cmd_vy = msg.vy
        cmd_vz = msg.vz
        cmd_yaw_rate = msg.yaw_rate

        if cmd_vx !=None and cmd_vy !=None and cmd_vz !=None and cmd_yaw_rate !=None:
            self.mavSendVelocityCMD(self.master, cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate)
            print("Velocity command received")
        else:
            print("No velocity command received")
   
    def mavSendVelocityCMD(self, master, vx:float, 
        vy:float, vz:float, yaw:float):
        """sends velocity commands in NED world frame"""        
        master.mav.set_position_target_local_ned_send(
            0, #time_boot_ms
            master.target_system, #target_system
            master.target_component, #target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, #coordinate frame
            0b0000111111000111, #type_mask #enable vx, vy, vz, afx, afy, afz
            # 0b110111111000, #type_mask #enable position
            0, #x
            0, #y
            0, #z
            vx, #vx
            vy, #vy
            0, #vz
            0, #afx
            0, #afy
            0, #afz
            0, #yaw
            0) #yaw_rate 

        return 

def main(args=None):
    rclpy.init(args=args)

    ip_string = '127.0.0.1:14551'
    master = mavutil.mavlink_connection(ip_string)
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    drone = DroneNode(master)
    drone.publishData()

    while rclpy.ok():
        drone.publishData()
        # drone.mavSendVelocityCMD(master, -5, -5, 5, 0)
        rclpy.spin_once(drone, timeout_sec=1/drone.frequency)

if __name__ == '__main__':
    main()



        
        


        
        


        
 


