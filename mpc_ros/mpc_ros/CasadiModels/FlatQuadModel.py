#!/usr/bin/env python3
import casadi as ca
import numpy as np

class FlatQuadcopterModel():
    def __init__(self):
        #model constants for dj100 from paper
        self.k_x = 1 
        self.k_y = 1 
        self.k_z = 1 
        self.k_psi = np.pi/180
        
        #tau constaints
        self.tau_x = 0.8355
        self.tau_y = 0.7701
        self.tau_z = 0.5013
        self.tau_psi = 0.5142 
        
        self.define_states()
        self.define_controls()
        self.state_history = {
            'x': [],
            'y': [],
            'z': [],
            'psi': [],
            'vx': [],
            'vy': [],
            'vz': [],
            'psi_dot': []
        }
        
    def define_states(self) -> None:
        """
        define the 8 flat states of the quadcopter
        
        [x, y, z, psi, vx, vy, vz, psi_dot]
        
        """
        
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')
        self.z = ca.SX.sym('z')
        self.psi = ca.SX.sym('psi')
        
        self.vx = ca.SX.sym('vx')
        self.vy = ca.SX.sym('vy')
        self.vz = ca.SX.sym('vz')
        self.psi_dot = ca.SX.sym('psi')
        
        self.states = ca.vertcat(
            self.x, 
            self.y,
            self.z,
            self.psi,
            self.vx,
            self.vy,
            self.vz, 
            self.psi_dot
        )
        
        #column vector of 3 x 1
        self.n_states = self.states.size()[0] #is a column vector 

    def define_controls(self) -> None:
        """4 motors"""
        self.u_0 = ca.SX.sym('u_0')
        self.u_1 = ca.SX.sym('u_1')
        self.u_2 = ca.SX.sym('u_2')
        self.u_3 = ca.SX.sym('u_3')
        
        self.controls = ca.vertcat(
            self.u_0,
            self.u_1,
            self.u_2,
            self.u_3
        )
        
        #column vector of 2 x 1
        self.n_controls = self.controls.size()[0] 
        
    def set_state_space(self) -> None:
        #this is where I do the dynamics for state space
        magnitude = ca.sqrt(self.vx**2 + self.vy**2)
        self.z_0 = self.vx * ca.cos(self.psi) - self.vy * ca.sin(self.psi)
        self.z_1 = self.vy * ca.sin(self.psi) + self.vy * ca.cos(self.psi)
        self.z_2 = self.vz 
        self.z_3 = self.psi_dot
        
        self.x_ddot = (-self.vx + (self.k_x * self.u_0))
        self.y_ddot = (-self.vy + (self.k_y * self.u_1))
        self.z_ddot = (-self.vz + (self.k_z * self.u_2))
        self.psi_ddot = (-self.psi_dot + (self.k_psi * self.u_3))


        #renamed it as z because I have an x variable, avoid confusion    
        self.z_dot = ca.vertcat(
            self.z_0, 
            self.z_1, 
            self.z_2, 
            self.z_3,
            self.x_ddot, 
            self.y_ddot, 
            self.z_ddot, 
            self.psi_ddot
        )
        
        #ODE right hand side function
        self.function = ca.Function('f', 
                        [self.states, self.controls],
                        [self.z_dot]
                        ) 
        
        # return self.function

        