#!/usr/bin/env python3
import numpy as np

class PropNavigation():
    """
    this class runs the proportional navigation algorithm
    Equation is as follows:
        an = N * lambda_dot * Vc
        
    where:
        an = turn acceleration (rad/s^2)
        N = tuning parameter
        lambda_dot = rate of change of the bearing angle (rad/s)
        Vc = speed of the vehicle (m/s)    
    """

    def __init__(self, N = 10, dt = 0.1) -> None:
        self.N = N #tuning parameter
        self.dt = dt #time step
        self.LOS = [0,0] #previous and current line of sight

    def update_vals(self, target_heading, target_position):
        self.LOS[1] = self.LOS[0]
        self.LOS[0] = target_heading
        self.target_position = target_position 

    def get_turn_acceleration(self, target_pos, current_pos, 
                              target_heading, current_heading,
                              target_speed, current_speed) -> float:
        
        #compute angle between target_pos and current_pos
        target_heading = np.atan2(target_pos[1] - current_pos[1], target_pos[0] - current_pos[0])
        LOS_dot = (self.LOS[0] - self.LOS[1]) / self.dt
        self.LOS[1] = self.LOS[0]
        self.LOS[0] = target_heading
        an = self.N * LOS_dot * current_speed

        return  self.N * LOS_dot       


