import math
import numpy as np

def RzVector(dimmention: str, degrees, vector):
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

def translateVector(matrixA, matrixB):
    return matrixA + matrixB
    
def transformVector(dimmention: str, degrees, initMat, displaceMat):
    return translateVector(RzVector(dimmention, degrees, initMat), displaceMat)
"""
Aquí empieza la actividad de la clase: 
"""
def translateMatrix (dimmention, val):
    tMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
    if (dimmention in ["x", "X"]):
        tMatrix[0][3] = val
    elif (dimmention in ["y", "Y"]):
        tMatrix[1][3] = val
    elif (dimmention in ["z", "Z"]):
        tMatrix[2][3] = val
    else:
        return ("ERROR")

    return tMatrix

def rotateMatrix (dimmention, degrees):

    angle = math.radians(degrees)
    zMatrix = []
    if (dimmention in ["x", "X"]):
        zMatrix = np.array([
            [1,0,0],
            [0,math.cos(angle), -math.sin(angle)],
            [0,math.sin(angle),math.cos(angle)]])
    elif (dimmention in ["y", "Y"]):
        zMatrix = np.array([
            [math.cos(angle), 0, math.sin(angle)],
            [0, 1, 0],
            [-math.sin(angle), 0, math.cos(angle)]])
    elif (dimmention in ["z", "Z"]):
        zMatrix = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1]])
    shape = np.shape(zMatrix)
    padded_array = np.zeros((4, 4))
    padded_array[:shape[0],:shape[1]] = zMatrix
    padded_array[3][3] = 1

    return padded_array
# TODO:
'''
Verify matrix dimentions
verify vector dimentions
def transformationmatrix

sympy
'''

theta1 = 45
theta2 = 90
a1 = 1
a2 = 1
                  # theta, d, a, alfa
joints = np.array([[theta1, 0, 0, 0],
                   [theta2, 0, a1, 0],
                   [0, 0, a2, 0]])

res = 1
print("+++++++++++")
for joint in joints:
    A_Joint = rotateMatrix("z", joint[0]) * translateMatrix("z", joint[1])* translateMatrix("x", joint[2]) * rotateMatrix("x", joint[3])       
    res = A_Joint*res
    print(A_Joint)
print("--------")
print(res)

