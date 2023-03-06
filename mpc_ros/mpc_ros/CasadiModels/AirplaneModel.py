#!/usr/bin/env python3
import casadi as ca

"""
Path-following control for small fixed-wing unmanned aerial vehicles under wind disturbances
"""


class AirplaneSimpleModel():
    def __init__(self):
        self.define_states()
        self.define_controls()

    def define_states(self):
        """define the states of your system"""
        # positions ofrom world
        self.x_f = ca.SX.sym('x_f')
        self.y_f = ca.SX.sym('y_f')
        self.z_f = ca.SX.sym('z_f')

        # attitude
        self.phi_f = ca.SX.sym('phi_f')
        self.theta_f = ca.SX.sym('theta_f')
        self.psi_f = ca.SX.sym('psi_f')
        self.airspeed = ca.SX.sym('airspeed')

        self.states = ca.vertcat(
            self.x_f,
            self.y_f,
            self.z_f,
            self.phi_f,
            self.theta_f,
            self.psi_f,
            self.airspeed
        )

        self.n_states = self.states.size()[0]  # is a column vector

    def define_controls(self):
        """controls for your system"""
        self.u_phi = ca.SX.sym('u_phi')
        self.u_theta = ca.SX.sym('u_theta')
        self.u_psi = ca.SX.sym('u_psi')
        self.v_cmd = ca.SX.sym('v_cmd')

        self.controls = ca.vertcat(
            self.u_phi,
            self.u_theta,
            self.u_psi,
            self.v_cmd
        )
        self.n_controls = self.controls.size()[0]

    def set_state_space(self):
        """define the state space of your system"""
        self.g = 9.81  # m/s^2
        # body to inertia frame
        #self.x_fdot = self.v_cmd *  ca.cos(self.theta_f) * ca.cos(self.psi_f)
        self.x_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.cos(self.psi_f)
        self.y_fdot = self.v_cmd * ca.cos(self.theta_f) * ca.sin(self.psi_f)
        self.z_fdot = -self.v_cmd * ca.sin(self.theta_f)

        self.phi_fdot = self.u_phi
        self.theta_fdot = self.u_theta
        ###!!!!!! From the PAPER ADD A NEGATIVE SIN BECAUSE OF SIGN CONVENTION!!!!!!!###
        self.psi_fdot = -self.g * (ca.tan(self.phi_f) / self.v_cmd)
        self.airspeed_fdot = self.v_cmd # u  


        self.z_dot = ca.vertcat(
            self.x_fdot,
            self.y_fdot,
            self.z_fdot,
            self.phi_fdot,
            self.theta_fdot,
            self.psi_fdot,
            self.airspeed_fdot
        )

        # ODE function
        self.function = ca.Function('f',
                                    [self.states, self.controls],
                                    [self.z_dot])


