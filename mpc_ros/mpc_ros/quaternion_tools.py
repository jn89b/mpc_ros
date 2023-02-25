import math  
import numpy as np
#https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/

def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def vector_euler_from_quaternion(x:np.ndarray, y:np.ndarray, z:np.ndarray, w:np.ndarray):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def get_quaternion_from_euler(roll:float, pitch:float, yaw:float) -> list:
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


#rotation 3d
def rot3d(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_x, np.dot(R_y, R_z))
    return R

def rot2d(psi):
    return np.array([[np.cos(psi), -np.sin(psi)],
                     [np.sin(psi), np.cos(psi)]])

def convertENUToNED(x_enu:float, y_enu:float, z_enu:float) -> list:
    """converts from ENU to NED"""
    ned =  np.zeros(3, dtype=np.float64)
    ned[0] = y_enu
    ned[1] = x_enu
    ned[2] = -z_enu
    return ned

def convertNEDToENU(x_ned:float, y_ned:float, z_ned:float) -> list:
    """converts from NED to ENU"""
    #create 3,1 array
    enu = np.zeros(3, dtype=np.float64)
    enu[0] = y_ned
    enu[1] = x_ned
    enu[2] = -z_ned
    return enu

def convertNEDtoENUVector(x_ned:np.ndarray, y_ned:np.ndarray, z_ned:np.ndarray) -> list:
    """converts from NED to ENU"""
    enu_x = y_ned
    enu_y = x_ned
    enu_z = -z_ned
    return enu_x, enu_y, enu_z

def convertENUToNEDVector(x_enu:np.ndarray, y_enu:np.ndarray, z_enu:np.ndarray) -> list:
    """converts from ENU to NED"""
    ned_x = y_enu
    ned_y = x_enu
    ned_z = -z_enu
    return ned_x, ned_y, ned_z  
