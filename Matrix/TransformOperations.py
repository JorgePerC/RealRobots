from logging import error
import math
import numpy as np

def Rz(dimmention: str, degrees, vector):
    angle = math.radians(degrees)
    zMatrix = 0
    if (dimmention in ["x", "X"]):
        zMatrix = np.array(
            [1,0,0],
            [0,math.cos(angle), -math.sin(angle)],
            [0,math.sin(angle),math.cos(angle)])
    elif (dimmention in ["y", "Y"]):
        zMatrix = np.array(
            [math.cos(angle), 0, math.sin(angle)],
            [0, 1, 0],
            [-math.sin(angle), 0, math.cos(angle)])
    elif (dimmention in ["z", "Z"]):
        zMatrix = np.array(
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle),0],
            [0, 0, 1])
    else:
        return ("ERROR")
    return zMatrix.dot(vector)

def translate(matrixA, matrixB):
    return matrixA + matrixB
    
def Transform(dimmention: str, degrees, initMat, displaceMat):
    return Rz(dimmention, degrees, initMat) + displaceMat

# TODO:
'''
Verify matrix dimentions
verify vector dimentions
def transformationmatrix
'''