#!/usr/bin/python3

from math import sin, cos, pi
import numpy as np

def getJacobianPlatform():
    return getJacobianPlatformWithoutEEF()

def getJacobianPlatformWithoutEEF():
    """get Jacobian of platform with coordinates x,theta

    Returns:
        Jacobian (np.array((6,2)))
    """
    J = np.zeros((6, 2))
    J[0,0]=1
    J[5,1]=1
    return J

def getJacobianPlatformWithEEF():
    """get Jacobian of platform with coordinates x,y,theta

    Returns:
        Jacobian (np.array((6,3)))
    """
    J = np.zeros((6, 3))
    J[0,0]=1
    J[1,1]=1
    J[5,2]=1
    return J

def getRotationMatrix(theta):
    R = np.zeros((3, 3))
    R[0,0]=cos(theta)
    R[0,1]=-sin(theta)
    R[1,0]=sin(theta)
    R[1,1]=cos(theta)
    R[2,2]=1
    return R

def getRotationMatrixS3(theta):
    """gets rotation matrix in S3

    Args:
        theta (float): robot direction

    Returns:
        np.array(6,6): Rotation matrix for S3 (R3 and SO3)
    """
    R6 = np.zeros((6, 6))
    R6[0:3,0:3]=getRotationMatrix(theta)
    R6[3:6,3:6]=getRotationMatrix(theta)
    return R6

def getJacobianPlatformToMap(theta):
    # J = np.zeros((6, 2))
    # J[0,0]=cos(theta)
    # J[1,0]=sin(theta)
    # J[5,1]=1
    J=getRotationMatrixS3(theta)@getJacobianPlatform()
    return J

if __name__ == "__main__":
    theta = pi/2
    print(getJacobianPlatform(theta))