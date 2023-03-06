#!/usr/bin/env python3
import casadi as ca
import numpy as np

class SimpleQuadModel():
    def __init__(self):
        
        self.define_states()
        self.define_controls()

    def define_states(self):
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')
        self.z = ca.SX.sym('z')
        self.psi = ca.SX.sym('psi')
        self.states = ca.vertcat(
            self.x, 
            self.y,
            self.z,
            self.psi,
        )

        self.n_states = self.states.shape[0]

    def define_controls(self):
        self.vx = ca.SX.sym('vx')
        self.vy = ca.SX.sym('vy')
        self.vz = ca.SX.sym('vz')
        self.psi_dot = ca.SX.sym('psi_dot')

        self.controls = ca.vertcat(
            self.vx,
            self.vy,
            self.vz,
            self.psi_dot
        )

        self.n_controls = self.controls.shape[0]

    def set_state_space(self):
        # self.magnitude = ca.sqrt(self.vx**2 + self.vy**2)
        self.z_0 = self.vx #* ca.cos(self.psi) + self.vy * ca.sin(self.psi)  
        self.z_1 = self.vy #* ca.cos(self.psi) - self.vx * ca.sin(self.psi)
        self.z_2 = self.vz 
        self.z_3 =  ca.atan2(self.x, self.y)

        self.z_dot = ca.vertcat(
            self.z_0,
            self.z_1,
            self.z_2,
            self.z_3,
        )

        self.function = ca.Function('f',
            [self.states, self.controls],
            [self.z_dot]
            )


