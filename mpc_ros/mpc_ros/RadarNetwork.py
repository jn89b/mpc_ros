import numpy as np
"""
RADAR SYSTEMS WITH MATLAB 
Chapters 13 and Chapter 
Lit review led me to here 
https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4237131 
http://dsp-book.narod.ru/RSAD/C1828_PDF_C02.pdf
https://kgut.ac.ir/useruploads/1581935317812mst.pdf

#https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7979453

"""
import casadi as ca
import numpy as np
import seaborn as sns
import pandas as pd
import scipy.interpolate as interpolate
from mpc_ros import Config

class Radar():
    def __init__(self, position:list, heading:float, 
                 c1=1.01, 
                 c2=1E-12) -> None:
        self.position = position
        self.heading = heading #
        self.elevation = 0  
        self.range = 750
        self.c1 = c1
        self.c2 = c2

    def compute_distance_from_radar(self, target_position:list, use_casadi=False):
        """
        radar_position : numpy array of shape (3,)
        target_position : numpy array of shape (3,)
        """
        
        if use_casadi == True:
            if not isinstance(self.position, ca.DM):
                self.position = ca.DM(self.position)

            distance = ca.norm_2(self.position - target_position)
            return distance

        if not isinstance(self.position, np.ndarray):
            self.position = np.array(self.position)
        if not isinstance(target_position, np.ndarray):
            target_position = np.array(target_position)

        distance = np.linalg.norm(self.position - target_position)
        return distance
    
    def compute_relative_heading(self, target_heading:float, use_casadi=False):
        """
        computes the relative heading between the radar and the target
        """

        if use_casadi == True:
            if not isinstance(self.heading, ca.DM):
                self.heading = ca.DM([self.heading])

            relative_heading = target_heading - self.heading
            relative_heading = ca.fmod(relative_heading, ca.pi*2)
            return relative_heading

        if not isinstance(self.heading, np.ndarray):
            self.heading = np.array(self.heading)
        if not isinstance(target_heading, np.ndarray):
            target_heading = np.array(target_heading)

        relative_heading = target_heading - self.heading
        relative_heading = np.mod(relative_heading, 2*np.pi)
        return relative_heading

    def compute_relative_elevation(self, target_elevation:float, use_casadi=False):
        """
        computes the relative elevation between the radar and the target
        """
        if use_casadi == True:
            if not isinstance(self.elevation, ca.DM):
                self.elevation = ca.DM([self.elevation])

            relative_elevation = target_elevation - self.elevation
            return relative_elevation
        
        if not isinstance(self.elevation, np.ndarray):
            self.elevation = np.array(self.elevation)
        if not isinstance(target_elevation, np.ndarray):
            target_elevation = np.array(target_elevation)
            
        relative_elevation = target_elevation - self.elevation
        relative_elevation = np.mod(relative_elevation, 2*np.pi)
        return relative_elevation


    def compute_instant_probability_detection(self, sigma:float, r:float, use_casadi=False):
        if use_casadi == True:
            pass
        bottom = 1 + (self.c2*r**4/sigma)**self.c1
        return 1/bottom

class RadarNetwork():
    def __init__(self, 
                 set_random=False, 
                 network_factor=0.9, 
                 decision_threshold=0.45) -> None:
        
        self.radars = self.init_radars(set_random=set_random)
        self.network_factor = network_factor
        self.decision_threshold = decision_threshold
        self.K0 = network_factor*np.sqrt(len(self.radars))
        # self.rcs_equation = get_rcs_equation('flat_toy_rcs')

        """
        Did one where sides a < b and c
        Did one where a > b and c
        
        """

        self.a = 0.068
        self.b = 0.05
        self.c = 0.05

    def init_radars(self, set_random=False) -> list:
        """initializes the radars"""
        if set_random == False:

            radar_position = [
                [500,500, 0]
                # [750, 750, 0]
                # [600,600,0]
                ]

            radar_heading = np.linspace(0, 0, len(radar_position))
        else:
            radar_position = []
            radar_heading = []
            for i in range(6):
                radar_position.append([np.random.randint(0, 1500), np.random.randint(0, 1500), 0])
                radar_heading.append(np.random.randint(0, 360))
            
        radars = []
        for position, heading in zip(radar_position, radar_heading):
            radar = Radar(position, heading)
            radars.append(radar)

        return radars

    def compute_radar_probability(self, prob_instant_array, use_casadi=Config.RADAR_USE_CASADI) -> float:
        #check if prob_instant_array is a numpy array
    

        if use_casadi == True:    
            #compute mean of the array
            mean_prob_instant = ca.sum1(prob_instant_array)/prob_instant_array.size1()
            prob_detection = mean_prob_instant
            
            return prob_detection
        

        if not isinstance(prob_instant_array, np.ndarray):
            prob_instant_array = np.array(prob_instant_array)

        mean_prob_instant = np.mean(prob_instant_array)
        prob_detection = 1 - np.prod(1 - mean_prob_instant)
        return prob_detection

    def get_distance_headings(self, target_position:float, target_heading:float, target_elevation:float, 
                              use_casadi=Config.RADAR_USE_CASADI) -> tuple:
        """computes the distance and heading from each radar to the target"""
        distances_from_radar = []
        relative_headings = []
        relative_elevations = []
        for radar in self.radars:
            
            distance = radar.compute_distance_from_radar(target_position, use_casadi=use_casadi)
            distances_from_radar.append(distance)
            
            relative_heading = radar.compute_relative_heading(target_heading, use_casadi=use_casadi)
            relative_headings.append(relative_heading)
            
            relative_elvation = radar.compute_relative_elevation(target_elevation, use_casadi=use_casadi)
            relative_elevations.append(relative_elvation)

        return distances_from_radar, relative_headings, relative_elevations
    
    def get_detection_values(self, distances_from_radar:list, relative_headings:list, 
                             relative_elevations:list, 
                             use_casadi=Config.RADAR_USE_CASADI) -> list:

        detection_values = []
        if use_casadi == True:
            detection_values = ca.DM.zeros(len(self.radars)) 

            for radar, distance, relative_heading, relative_ele in zip(
                    self.radars, distances_from_radar, relative_headings, relative_elevations):

                out_of_range = distance > radar.range
                rcs = compute_dynamic_rcs(self.a, self.b, self.c, relative_heading, relative_ele)
                result = ca.if_else(out_of_range, 0,  radar.compute_instant_probability_detection(
                    rcs, distance, use_casadi=use_casadi))
                detection_values = ca.vertcat(detection_values, result)                

            return detection_values

        print("relative elevations: ", relative_elevations)

        for radar, distance, relative_heading, relative_ele in zip(
                self.radars, distances_from_radar, relative_headings, relative_elevations):
            if distance > radar.range:
                detection_values.append(0)
                continue
            else:
                #rcs = self.rcs_equation(relative_heading)
                rcs = compute_dynamic_rcs(self.a, self.b, self.c, relative_heading, relative_ele)
                detection_value = radar.compute_instant_probability_detection(
                    rcs, distance)
                detection_values.append(detection_value)

        return detection_values
    
    
    def get_detection_probability(self, target_position:list, target_heading:float, target_elevation:float) -> tuple:
        """computes the decision network for each radar"""
        
        distances_from_radar, relative_headings, relative_elevations = self.get_distance_headings(
            target_position, target_heading, target_elevation)
                
        detection_values = self.get_detection_values(distances_from_radar, relative_headings, relative_elevations)

        prob_detection = self.compute_radar_probability(detection_values)
        # print('Probability of Detection: ', prob_detection)

        return detection_values, prob_detection

        # is_detected_list = []
        # for prob in prob_detection:
        #     if prob > self.decision_threshold:
        #         is_detected_list.append(1)
        #     else:
        #         is_detected_list.append(0)

        # sum_decision = sum(is_detected_list)

        # if sum_decision >= self.K0:
        #     return True, sum_decision, prob_detection
        # else:
        #     return False, sum_decision, prob_detection


def get_rcs_equation(csv_file:str):
    df = pd.read_csv(csv_file+'.csv')
    #print columns 
    azimith_angles = df.columns[1:]
    #convert string to float
    azimith_angles = [float(x) for x in azimith_angles]
    reflected_angle = [x + 180 for x in azimith_angles]
    azimith_angles = azimith_angles + reflected_angle

    #remove first column from df
    #sort rows by ascending value of first column
    df = df.sort_values(by=df.columns[0])
    df = df.iloc[:, 1:]
    #convert to numpy array
    rcs_array = df.to_numpy()
    rcs_array = np.transpose(rcs_array)
    flipped_df = np.flip(rcs_array, axis=0)
    rcs_array = np.concatenate((rcs_array, flipped_df), axis=0)
    rcs_array = rcs_array.reshape(-1)
    rcs_y_interpolate = interpolate.interp1d(np.deg2rad(azimith_angles), rcs_array, axis=0, kind='linear')
    
    test_angles = np.linspace(0,2*np.pi,100)
    test_vals = rcs_y_interpolate(test_angles)

    rcs_interpolate = ca.interpolant('rcs_interpolate', 'linear', [test_angles], test_vals)

    return rcs_interpolate

def compute_dynamic_rcs(a:float,b:float,c:float,
                        aspect_angle:float, 
                        roll_angle:float):
    """
    a is front of aircraft
    b is side of aircraft
    c is top of aircraft 

    aspect_angle : radians
    roll_angle : radians  

    
    RADAR SYSTEMS WITH MATLAB 
    Chapters 13 and Chapter 
    Lit review led me to here 
    https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4237131 
    http://dsp-book.narod.ru/RSAD/C1828_PDF_C02.pdf
    https://kgut.ac.ir/useruploads/1581935317812mst.pdf
    
    #https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7979453
    """
    
    c1 = a**2 *(np.sin(aspect_angle))**2 * np.cos(roll_angle)**2 
    c2 = b**2 *np.sin(aspect_angle)**2 * np.sin(roll_angle)**2
    c3 = c**2 *np.cos(aspect_angle)**2
    
    top = np.pi*a**2*b**2*c**2
    bottom = (c1 + c2 + c3)**2

    return top/bottom


        